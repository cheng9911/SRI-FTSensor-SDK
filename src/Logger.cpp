#include "sri/Logger.h"

#include "spdlog/async.h"

using namespace std;
using namespace spdlog;

Logger::Logger(string name) {
    name_ = name;
    if(!isThreadPoolInit) {
        init_thread_pool(8192, 1);
        isThreadPoolInit = true;
    }
    logger_ptr_ = make_shared<async_logger>(name_, sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
    register_logger(logger_ptr_);

    // logger_ptr_->set_level(spdlog::level::err);
    logger_ptr_->flush_on(spdlog::level::info);

    // spdlog::flush_every(std::chrono::milliseconds(100));
}

Logger::~Logger() {
    drop(name_); // 清理日志记录器
}


// 创建每日轮换的异步日志接收器
sink_ptr Logger::daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>("logs/BoneCutting", 23, 59);
// 创建控制台接收器
sink_ptr Logger::console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

std::vector<sink_ptr> Logger::sinks = {daily_sink};

bool Logger::isThreadPoolInit = false;

Logger::logger_ptr Logger::getInstance(const string &name) {
        auto log = make_shared<Logger>(name);
        return log->logger_ptr_;
}
