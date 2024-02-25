#pragma once

#include <iostream>
#include <fstream>


/**
 * @brief 切换 Windows 命令行编码为 UTF-8
 * @see https://docs.microsoft.com/en-us/windows-server/administration/windows-commands/chcp
 */
void use_utf8() {
  system("chcp 65001");
}


/**
 * @brief 控制对象的存储
 * @tparam dType - 要序列化/反序列化的数据类型
 */
template<typename dType>
class BinFile {

public:
    /** @param file - 要与 BinFile 对象关联的文件路径 */
    BinFile(const std::string file) : file_(file) {}

    /** @brief 序列化函数, 用于将对象序列化并写入文件 */
    void dump(const dType &obj) {
      std::fstream f = this->open(std::ios::out);
      if (f) {
        f << (const char *) &obj;
        f.close();
      }
    }

    /** @brief 反序列化函数, 用于从文件中读取对象 */
    void load(dType &obj) {
      std::fstream f = this->open(std::ios::in);
      if (f) {
        f.read(reinterpret_cast<char *>(&obj), sizeof(obj));
        f.close();
      }
    }

protected:
    std::string file_;

    std::fstream open(std::ios::openmode mode) {
      std::fstream f(file_, mode | std::ios::binary);
      // 检查文件打开状态
      if (!f)
        std::cerr << "Failed to open file." << std::endl;
      return f;
    }

    friend std::ostream &operator<<(std::ostream &os, const BinFile &obj) {
      os << "BinFile<" << obj.file_ << ">";
      return os;
    }
};
