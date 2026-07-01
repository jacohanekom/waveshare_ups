/**
 * waveshare_ups.cpp
 * =================
 * Reads the Waveshare UPS HAT (B) via I2C (INA219 power monitor chip) and
 * broadcasts one JSON telemetry line per poll cycle over TCP — mirroring
 * the picam-hailo / victron-ve-direct network interface exactly.
 *
 *   DataServer   TCP data_port=8564  newline-delimited JSON, one per poll
 *   StatusServer TCP ctrl_port=8565  key=value reply to "status\n"
 *
 * JSON output (stdout + DataServer broadcast):
 *   {"ts_us":T,"frame":N,
 *    "V":3.820,"I":-0.550,"P":2.100,
 *    "V_shunt_mV":55.0,
 *    "SOC":75.0,"charging":false,"online":true}
 *
 * INA219 register map (16-bit big-endian, I2C):
 *   0x00  Configuration
 *   0x01  Shunt voltage   (signed, 10 µV/LSB)
 *   0x02  Bus voltage     (bits[15:3], 4 mV/LSB; bit1=CNVR, bit0=OVF)
 *   0x03  Power           (unsigned, power_LSB = 20 × current_LSB)
 *   0x04  Current         (signed, current_LSB configurable via calibration)
 *   0x05  Calibration     Cal = trunc(0.04096 / (current_LSB × R_shunt))
 *
 * Build (Linux / Raspberry Pi):
 *   cmake -B build && cmake --build build -j$(nproc)
 * Run:
 *   ./build/waveshare_ups [--config config.ini]
 * Stream data:
 *   nc 127.0.0.1 8564
 * Query status:
 *   echo status | nc 127.0.0.1 8565
 */

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "config.hpp"

using Clock     = std::chrono::steady_clock;
using SysClock  = std::chrono::system_clock;

// ─────────────────────────────────────────────────────────────────────────────
// Global stop flag
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_stop{false};
static void signal_handler(int) { g_stop = true; }

// ─────────────────────────────────────────────────────────────────────────────
// INA219 register addresses
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t INA219_REG_CONFIG    = 0x00;
static constexpr uint8_t INA219_REG_SHUNT_V   = 0x01;
static constexpr uint8_t INA219_REG_BUS_V     = 0x02;
static constexpr uint8_t INA219_REG_POWER     = 0x03;
static constexpr uint8_t INA219_REG_CURRENT   = 0x04;
static constexpr uint8_t INA219_REG_CALIB     = 0x05;

// Config register value:
//   BRNG=1 (32 V range), PGA=11 (÷8, ±320 mV), BADC=1101 (12-bit, 32 samples),
//   SADC=1101 (12-bit, 32 samples), MODE=111 (shunt+bus continuous)
// Matches Waveshare's reference INA219 driver (set_calibration_32V_2A).
static constexpr uint16_t INA219_CONFIG = 0x3EEF;

// ─────────────────────────────────────────────────────────────────────────────
// I2C helpers
// ─────────────────────────────────────────────────────────────────────────────
static bool i2c_write16(int fd, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg,
                       static_cast<uint8_t>(val >> 8),
                       static_cast<uint8_t>(val & 0xFF) };
    return ::write(fd, buf, 3) == 3;
}

static bool i2c_read16(int fd, uint8_t reg, uint16_t &out) {
    if (::write(fd, &reg, 1) != 1) return false;
    uint8_t buf[2];
    if (::read(fd, buf, 2) != 2) return false;
    out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// INA219 — open, configure, read
// ─────────────────────────────────────────────────────────────────────────────
struct INA219Config {
    double r_shunt     = 0.1;      // Ohm
    double current_lsb = 0.0001;   // A/bit  (0.1 mA resolution)
};

static int ina219_open(const std::string &bus, int addr) {
    int fd = ::open(bus.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "[I2C] open(" << bus << "): " << strerror(errno) << "\n";
        return -1;
    }
    if (::ioctl(fd, I2C_SLAVE, addr) < 0) {
        std::cerr << "[I2C] ioctl(I2C_SLAVE, 0x" << std::hex << addr
                  << std::dec << "): " << strerror(errno) << "\n";
        ::close(fd);
        return -1;
    }
    return fd;
}

static bool ina219_configure(int fd, const INA219Config &cfg) {
    if (!i2c_write16(fd, INA219_REG_CONFIG, INA219_CONFIG)) {
        std::cerr << "[I2C] write config failed\n";
        return false;
    }
    // Cal = trunc(0.04096 / (current_lsb * r_shunt))
    auto cal = static_cast<uint16_t>(0.04096 / (cfg.current_lsb * cfg.r_shunt));
    if (!i2c_write16(fd, INA219_REG_CALIB, cal)) {
        std::cerr << "[I2C] write calibration failed\n";
        return false;
    }
    std::cerr << "[INA219] configured: r_shunt=" << cfg.r_shunt
              << "Ω  current_lsb=" << cfg.current_lsb * 1000.0
              << "mA  cal=0x" << std::hex << cal << std::dec << "\n";
    return true;
}

struct INA219Reading {
    double V        = 0;   // bus voltage V
    double V_shunt  = 0;   // shunt voltage mV
    double I        = 0;   // current A (positive = charging)
    double P        = 0;   // power W
    bool   overflow = false;
    bool   ready    = false;
};

static bool ina219_read(int fd, const INA219Config &cfg, INA219Reading &r) {
    uint16_t raw_bus, raw_shunt, raw_current, raw_power;

    if (!i2c_read16(fd, INA219_REG_BUS_V,   raw_bus))     return false;
    if (!i2c_read16(fd, INA219_REG_SHUNT_V, raw_shunt))   return false;
    if (!i2c_read16(fd, INA219_REG_CURRENT, raw_current)) return false;
    if (!i2c_read16(fd, INA219_REG_POWER,   raw_power))   return false;

    r.ready    = (raw_bus & 0x0002) != 0;   // CNVR bit
    r.overflow = (raw_bus & 0x0001) != 0;   // OVF bit

    // Bus voltage: bits[15:3], 4 mV/LSB
    r.V = static_cast<double>(raw_bus >> 3) * 0.004;

    // Shunt voltage: signed 16-bit, 10 µV/LSB → mV
    r.V_shunt = static_cast<double>(static_cast<int16_t>(raw_shunt)) * 0.01;

    // Current: signed, current_lsb A/bit
    r.I = static_cast<double>(static_cast<int16_t>(raw_current)) * cfg.current_lsb;

    // Power: unsigned, 20 × current_lsb W/bit
    r.P = static_cast<double>(raw_power) * 20.0 * cfg.current_lsb;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// SOC estimation from bus voltage
//
// Linear interpolation between the configured empty and full voltages.
// For a single Li-Ion cell: empty=3.0 V, full=4.2 V.
// Clamps to [0, 100].
// ─────────────────────────────────────────────────────────────────────────────
static double voltage_to_soc(double v, double v_empty, double v_full) {
    if (v <= v_empty) return 0.0;
    if (v >= v_full)  return 100.0;
    return (v - v_empty) / (v_full - v_empty) * 100.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON formatting
// ─────────────────────────────────────────────────────────────────────────────
static std::string format_frame(uint64_t frame, int64_t ts_us,
                                const INA219Reading &r,
                                double soc, bool charging, bool online)
{
    std::string j;
    j.reserve(256);

    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"ts_us\":%lld,\"frame\":%llu"
        ",\"V\":%.3f,\"I\":%.3f,\"P\":%.3f"
        ",\"V_shunt_mV\":%.2f"
        ",\"SOC\":%.1f"
        ",\"charging\":%s"
        ",\"online\":%s}",
        (long long)ts_us,
        (unsigned long long)frame,
        r.V, r.I, r.P,
        r.V_shunt,
        soc,
        charging ? "true" : "false",
        online   ? "true" : "false");

    return std::string(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
// Shared status
// ─────────────────────────────────────────────────────────────────────────────
struct UpsStatus {
    std::mutex  mu;
    uint64_t    frame_count = 0;
    float       fps         = 0;
    double      V           = 0;
    double      I           = 0;
    double      P           = 0;
    double      V_shunt     = 0;
    double      SOC         = 0;
    bool        charging    = false;
    bool        online      = false;
    std::string bus;
    int         addr        = 0;
    std::string last_json;
};
static UpsStatus g_status;

// ─────────────────────────────────────────────────────────────────────────────
// StatusServer — TCP plain-text on ctrl_port
// ─────────────────────────────────────────────────────────────────────────────
class StatusServer {
public:
    explicit StatusServer(int port) : port_(port) {}
    ~StatusServer() { if (fd_ >= 0) ::close(fd_); }

    void start() {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(static_cast<uint16_t>(port_));
        ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        ::listen(fd_, 8);
        thread_ = std::thread(&StatusServer::acceptLoop, this);
        thread_.detach();
        std::cerr << "[Status] Listening on 0.0.0.0:" << port_ << "\n";
    }

private:
    void acceptLoop() {
        while (!g_stop) {
            int cfd = ::accept(fd_, nullptr, nullptr);
            if (cfd < 0) { if (errno == EINTR) continue; break; }
            std::thread([cfd]{ handle(cfd); }).detach();
        }
    }

    static void handle(int cfd) {
        struct timeval tv{2, 0};
        ::setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        std::string line;
        char c;
        while (::recv(cfd, &c, 1, 0) == 1) {
            if (c == '\n') break;
            if (c != '\r') line += c;
        }
        std::string reply;
        if (line == "status" || line.empty()) {
            std::lock_guard<std::mutex> lk(g_status.mu);
            char buf[1024];
            snprintf(buf, sizeof(buf),
                "ok=true\n"
                "frame=%llu\n"
                "fps=%.1f\n"
                "bus=%s\n"
                "addr=0x%02x\n"
                "V=%.3f\n"
                "I=%.3f\n"
                "P=%.3f\n"
                "V_shunt_mV=%.2f\n"
                "SOC=%.1f\n"
                "charging=%s\n"
                "online=%s\n"
                "\n",
                (unsigned long long)g_status.frame_count,
                g_status.fps,
                g_status.bus.c_str(),
                g_status.addr,
                g_status.V,
                g_status.I,
                g_status.P,
                g_status.V_shunt,
                g_status.SOC,
                g_status.charging ? "true" : "false",
                g_status.online   ? "true" : "false");
            reply = buf;
        } else {
            reply = "ok=false\nerror=unknown command\ncommands=status\n\n";
        }
        ::send(cfd, reply.data(), reply.size(), MSG_NOSIGNAL);
        ::close(cfd);
    }

    int         port_;
    int         fd_ = -1;
    std::thread thread_;
};

// ─────────────────────────────────────────────────────────────────────────────
// DataServer — TCP broadcast, one JSON line per poll
// ─────────────────────────────────────────────────────────────────────────────
class DataServer {
public:
    explicit DataServer(int port) : port_(port) {}
    ~DataServer() { if (listenFd_ >= 0) ::close(listenFd_); }

    void start() {
        listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(static_cast<uint16_t>(port_));
        ::bind(listenFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        ::listen(listenFd_, 16);
        thread_ = std::thread(&DataServer::acceptLoop, this);
        thread_.detach();
        std::cerr << "[Data] Listening on 0.0.0.0:" << port_ << "\n";
    }

    void broadcast(const std::string &line) {
        std::string msg = line + "\n";
        std::lock_guard<std::mutex> lk(mu_);
        std::vector<int> alive;
        for (int fd : clients_) {
            ssize_t n = ::send(fd, msg.data(), msg.size(),
                               MSG_NOSIGNAL | MSG_DONTWAIT);
            if (n > 0 || (n < 0 && errno == EAGAIN)) {
                alive.push_back(fd);
            } else {
                std::cerr << "\n[Data] Client fd=" << fd
                          << " dropped: " << strerror(errno) << "\n";
                ::close(fd);
            }
        }
        clients_ = std::move(alive);
    }

private:
    void acceptLoop() {
        while (!g_stop) {
            sockaddr_in peer{};
            socklen_t   plen = sizeof(peer);
            int cfd = ::accept(listenFd_,
                               reinterpret_cast<sockaddr*>(&peer), &plen);
            if (cfd < 0) { if (errno == EINTR) continue; break; }
            char ip[INET_ADDRSTRLEN];
            ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
            std::cerr << "\n[Data] Client: " << ip
                      << ":" << ntohs(peer.sin_port) << "\n";
            std::lock_guard<std::mutex> lk(mu_);
            clients_.push_back(cfd);
        }
    }

    int                port_;
    int                listenFd_ = -1;
    std::thread        thread_;
    mutable std::mutex mu_;
    std::vector<int>   clients_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGHUP,  signal_handler);
    std::signal(SIGPIPE, SIG_IGN);

    std::string cfg_path = "config.ini";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i + 1 < argc) cfg_path = argv[++i];
        else if (a == "--help" || a == "-h") {
            std::cout << "Usage: " << argv[0] << " [--config config.ini]\n"
                      << "  Stream data : nc 127.0.0.1 8564\n"
                      << "  Status      : echo status | nc 127.0.0.1 8565\n";
            return 0;
        }
    }

    Config cfg(cfg_path);

    const std::string i2c_bus   = cfg.get_str("device.bus",      "/dev/i2c-1");
    const int         i2c_addr  = cfg.get_int("device.addr",     0x42);
    const int         poll_ms   = cfg.get_int("device.poll_ms",  1000);
    const int         retry_s   = cfg.get_int("device.retry_secs", 5);
    const int         ctrl_port = cfg.get_int("output.ctrl_port", 8565);
    const int         data_port = cfg.get_int("output.data_port", 8564);

    INA219Config ina;
    ina.r_shunt     = cfg.get_double("ina219.r_shunt",     0.1);
    ina.current_lsb = cfg.get_double("ina219.current_lsb", 0.0001);

    const double v_empty = cfg.get_double("battery.v_empty", 3.0);
    const double v_full  = cfg.get_double("battery.v_full",  4.2);
    // Current threshold below which charging is considered "not charging" (A)
    const double charge_threshold = cfg.get_double("battery.charge_threshold", 0.05);

    {
        std::lock_guard<std::mutex> lk(g_status.mu);
        g_status.bus  = i2c_bus;
        g_status.addr = i2c_addr;
    }

    std::cerr << "[Config] i2c bus : " << i2c_bus
              << "  addr=0x" << std::hex << i2c_addr << std::dec << "\n"
              << "[Config] poll    : " << poll_ms << " ms\n"
              << "[Config] status  : 0.0.0.0:" << ctrl_port << "\n"
              << "[Config] data    : 0.0.0.0:" << data_port << "\n"
              << "[Config] battery : empty=" << v_empty << "V  full=" << v_full << "V\n";

    DataServer   data_srv(data_port);
    StatusServer status_srv(ctrl_port);
    data_srv.start();
    status_srv.start();

    std::cerr << "[Main] Running.\n"
              << "[Main]   Data stream : nc 127.0.0.1 " << data_port << "\n"
              << "[Main]   Status      : echo status | nc 127.0.0.1 " << ctrl_port << "\n";

    uint64_t frame_count   = 0;
    auto     t_start       = Clock::now();
    auto     t_last_fps    = Clock::now();
    uint64_t frames_since  = 0;

    while (!g_stop) {
        // Open / reopen I2C device
        int fd = ina219_open(i2c_bus, i2c_addr);
        if (fd < 0) {
            std::cerr << "[I2C] Retrying in " << retry_s << "s...\n";
            for (int i = 0; i < retry_s * 10 && !g_stop; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (!ina219_configure(fd, ina)) {
            ::close(fd);
            std::cerr << "[INA219] Configure failed. Retrying in " << retry_s << "s...\n";
            for (int i = 0; i < retry_s * 10 && !g_stop; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        std::cerr << "[I2C] Opened " << i2c_bus
                  << " addr=0x" << std::hex << i2c_addr << std::dec << "\n";

        // Allow the first conversion to complete before reading
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        bool io_ok = true;
        while (!g_stop && io_ok) {
            auto poll_start = Clock::now();

            INA219Reading reading;
            if (!ina219_read(fd, ina, reading)) {
                std::cerr << "\n[I2C] Read error: " << strerror(errno) << "\n";
                io_ok = false;
                break;
            }

            if (reading.overflow)
                std::cerr << "\n[INA219] ADC overflow — check shunt/range config\n";

            ++frame_count;
            ++frames_since;

            auto ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
                SysClock::now().time_since_epoch()).count();

            double soc      = voltage_to_soc(reading.V, v_empty, v_full);
            bool   charging = (reading.I >  charge_threshold);
            bool   online   = (reading.V >= v_empty);

            std::string json = format_frame(frame_count, ts_us,
                                            reading, soc, charging, online);

            std::cout << json << "\n" << std::flush;
            data_srv.broadcast(json);

            {
                std::lock_guard<std::mutex> lk(g_status.mu);
                g_status.frame_count = frame_count;
                g_status.V           = reading.V;
                g_status.I           = reading.I;
                g_status.P           = reading.P;
                g_status.V_shunt     = reading.V_shunt;
                g_status.SOC         = soc;
                g_status.charging    = charging;
                g_status.online      = online;
                g_status.last_json   = json;
            }

            // 1 Hz stderr stats
            auto now = Clock::now();
            double since = std::chrono::duration<double>(now - t_last_fps).count();
            if (since >= 1.0) {
                double fps     = frames_since / since;
                double avg_fps = frame_count /
                    std::chrono::duration<double>(now - t_start).count();
                {
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    g_status.fps = static_cast<float>(fps);
                }
                fprintf(stderr,
                    "\r[%6llu] fps:%.1f avg:%.1f  "
                    "V:%5.3fV  I:%+.3fA  P:%.3fW  SOC:%.1f%%  %s   ",
                    (unsigned long long)frame_count, fps, avg_fps,
                    reading.V, reading.I, reading.P, soc,
                    charging ? "CHARGING" : (online ? "discharging" : "OFFLINE"));
                fflush(stderr);
                t_last_fps   = now;
                frames_since = 0;
            }

            // Sleep for the remainder of the poll interval
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - poll_start).count();
            auto sleep_ms = poll_ms - static_cast<int>(elapsed);
            if (sleep_ms > 0 && !g_stop)
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }

        ::close(fd);
        if (g_stop) break;

        std::cerr << "\n[I2C] Disconnected. Retrying in " << retry_s << "s...\n";
        for (int i = 0; i < retry_s * 10 && !g_stop; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    fprintf(stderr, "\n");
    double total = std::chrono::duration<double>(Clock::now() - t_start).count();
    std::cerr << "[Main] Done. frames=" << frame_count
              << "  avg_fps=" << (total > 0 ? frame_count / total : 0) << "\n";
    return 0;
}
