/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef FASTLIO_YAML_IO_H_
#define FASTLIO_YAML_IO_H_

#include <yaml-cpp/yaml.h>

#include <cassert>
#include <string>

namespace fastlio {

/// 读取yaml配置文件的相关IO
class YAML_IO {
   public:
    YAML_IO(const std::string &path);

    YAML_IO() {}

    ~YAML_IO();

    inline bool IsOpened() const { return is_opened_; }

    /// 保存文件，不指明路径时，覆盖原文件
    bool Save(const std::string &path = "");

    // 查询是否存在
    bool HasValue(const std::string &key) const {
        assert(is_opened_);
        YAML::Node node = yaml_node_[key];
        if (yaml_node_[key]) {
            return true;
        }
        return false;
    }

    bool HasValue(const std::string &node, const std::string &key) const {
        assert(is_opened_);
        if (yaml_node_[node][key]) {
            return true;
        }
        return false;
    }

    /// 获取类型为T的参数值
    template <typename T>
    T GetValue(const std::string &key) const {
        assert(is_opened_);
        return yaml_node_[key].as<T>();
    }

    /// 获取在NODE下的key值
    template <typename T>
    T GetValue(const std::string &node, const std::string &key) const {
        assert(is_opened_);
        T res = yaml_node_[node][key].as<T>();
        return res;
    }

    /// 设定类型为T的参数值
    template <typename T>
    void SetValue(const std::string &key, const T &value) {
        yaml_node_[key] = value;
    }

    /// 设定NODE下的key值
    template <typename T>
    void SetValue(const std::string &node, const std::string &key, const T &value) {
        yaml_node_[node][key] = value;
    }

   private:
    std::string path_;
    bool is_opened_ = false;
    YAML::Node yaml_node_;
};

}  // namespace fastlio

#endif  // FASTLIO_YAML_IO_H_
