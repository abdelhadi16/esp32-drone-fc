#pragma once
// In-memory file system with the arduino-esp32 FS / FFat API used by flight_log.h
#include <map>
#include <memory>
#include <string>
#include <vector>
#define FILE_READ   "r"
#define FILE_WRITE  "w"
#define FILE_APPEND "a"
struct SimFs { std::map<std::string, std::vector<uint8_t>> files; size_t capacity = 9900 * 1024; bool failRemove = false; };
extern SimFs g_fs;
namespace fs {
struct FileImpl { std::string path; bool dir = false, write = false; size_t pos = 0; std::vector<std::string> list; size_t li = 0; bool open = true; };
class File {
 public:
  File(std::shared_ptr<FileImpl> p = nullptr) : p(p) {}
  size_t write(uint8_t b) { return write(&b, 1); }
  size_t write(const uint8_t *buf, size_t n) {
    if (!p || !p->write) return 0;
    size_t used = 0; for (auto &f : g_fs.files) used += f.second.size() + 4096;
    if (used + n > g_fs.capacity) return 0;
    auto &v = g_fs.files[p->path]; v.insert(v.end(), buf, buf + n); return n; }
  int read() { uint8_t b; return read(&b, 1) == 1 ? b : -1; }
  size_t read(uint8_t *buf, size_t n) {
    if (!p || p->write || p->dir) return 0;
    auto it = g_fs.files.find(p->path); if (it == g_fs.files.end()) return 0;
    size_t k = 0; while (k < n && p->pos < it->second.size()) buf[k++] = it->second[p->pos++]; return k; }
  size_t size() const { if (!p) return 0; auto it = g_fs.files.find(p->path); return it == g_fs.files.end() ? 0 : it->second.size(); }
  void close() { if (p) p->open = false; }
  operator bool() const { return p && p->open; }
  const char *name() const { if (!p) return ""; size_t s = p->path.rfind('/'); return p->path.c_str() + (s == std::string::npos ? 0 : s + 1); }
  File openNextFile(const char * = FILE_READ) {
    if (!p || !p->dir || p->li >= p->list.size()) return File();
    auto f = std::make_shared<FileImpl>(); f->path = p->list[p->li++]; return File(f); }
  std::shared_ptr<FileImpl> p;
};
class FS {
 public:
  File open(const char *path, const char *mode = FILE_READ, const bool = false) {
    std::string ps(path); auto f = std::make_shared<FileImpl>(); f->path = ps;
    if (ps == "/") { f->dir = true; for (auto &kv : g_fs.files) f->list.push_back(kv.first); return File(f); }
    if (mode[0] == 'w') { g_fs.files[ps].clear(); f->write = true; return File(f); }
    if (!g_fs.files.count(ps)) return File();
    return File(f); }
  bool exists(const char *path) { return g_fs.files.count(path) > 0; }
  bool remove(const char *path) { if (g_fs.failRemove) return false; return g_fs.files.erase(path) > 0; }
};
class F_Fat : public FS {
 public:
  bool ok = true;
  bool begin(bool = false, const char * = "/ffat", uint8_t = 10, const char * = "ffat") { return ok; }
  size_t totalBytes() { return g_fs.capacity; }
  size_t usedBytes() { size_t u = 0; for (auto &f : g_fs.files) u += f.second.size() + 4096; return u; }
  size_t freeBytes() { size_t u = usedBytes(); return u > g_fs.capacity ? 0 : g_fs.capacity - u; }
};
}
using fs::File; using fs::FS;
extern fs::F_Fat FFat;
