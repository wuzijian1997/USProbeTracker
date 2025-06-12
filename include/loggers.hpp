/**
 * @file logger.hpp
 * @author Paul Lee (ycleepaul@gmail.com)
 * @version 0.2
 * @date 2025-05-26
 */

#pragma once

#include <iostream>
#include <spdlog/async.h>
#include <spdlog/cfg/env.h>
#include <spdlog/fmt/ranges.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/hourly_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

inline bool gInitializedSpdlogThreadPool = false;

/**
 * @brief Set up a multi-sink spdlog logger with console and daily file sinks.
 * @param loggerName
 * @param setAsDefaultLogger
 */
static void setupMultiSinkSpdlog(const std::string &loggerName, const bool setAsDefaultLogger = true)
{
    if (!gInitializedSpdlogThreadPool) {
        spdlog::init_thread_pool(8192, 1);
        gInitializedSpdlogThreadPool = true;
    }
    spdlog::cfg::load_env_levels();

    try {
        // create loggers with 2 targets with different log levels and formats.
        // the console will show only warnings or errors, while the file will log all.
        const auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        stdout_sink->set_level(spdlog::level::debug);

        // Create a daily logger - a new file is created every day on 23:59
        const auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>("logs/"
                                                                                       + loggerName
                                                                                       + ".txt",
                                                                                   2,
                                                                                   0);
        file_sink->set_level(spdlog::level::debug);

        std::vector<spdlog::sink_ptr> sinks = {stdout_sink, file_sink};

        // async version
        const auto logger
            = std::make_shared<spdlog::async_logger>(loggerName,
                                                     sinks.begin(),
                                                     sinks.end(),
                                                     spdlog::thread_pool(),
                                                     spdlog::async_overflow_policy::block);

        // sync version
        // auto logger = std::make_shared<spdlog::logger>(
        //     loggerName,
        //     sinks.begin(),
        //     sinks.end());

        logger->set_pattern("[%H:%M:%S:%e] [%^%L%$] [tid:%t] %v");
        logger->set_level(spdlog::level::debug);
        logger->debug("Using spdlog version {}.{}.{}  !",
                      SPDLOG_VER_MAJOR,
                      SPDLOG_VER_MINOR,
                      SPDLOG_VER_PATCH);
        logger->info("Using to spdlog version {}.{}.{}  !",
                     SPDLOG_VER_MAJOR,
                     SPDLOG_VER_MINOR,
                     SPDLOG_VER_PATCH);
        logger->warn("Using to spdlog version {}.{}.{}  !",
                     SPDLOG_VER_MAJOR,
                     SPDLOG_VER_MINOR,
                     SPDLOG_VER_PATCH);

        if (setAsDefaultLogger) {
            spdlog::set_default_logger(logger);
        } else {
            spdlog::register_logger(logger);
        }

        // Flush policy: https://github.com/gabime/spdlog/wiki/Flush-policy
        spdlog::flush_every(std::chrono::seconds(2));
    } catch (const spdlog::spdlog_ex &ex) {
        std::cout << "Log init failed: " << ex.what() << std::endl;
    }
}

/**
 * @brief Set up a spdlog logger only with file sink.
 * @param loggerName
 */
static void setupDataSinkSpdlog(const std::string &loggerName)
{
    if (!gInitializedSpdlogThreadPool) {
        spdlog::init_thread_pool(8192, 1);
        gInitializedSpdlogThreadPool = true;
    }
    spdlog::cfg::load_env_levels();

    try {
        // Create a daily logger - a new file is created every day on 23:59
        const auto file_sink = std::make_shared<spdlog::sinks::hourly_file_sink_mt>(
            "logs/" + loggerName + ".txt");
        file_sink->set_level(spdlog::level::trace);

        // include HH:MM:SS in the log file name

        // async version
        const auto logger
            = std::make_shared<spdlog::async_logger>(loggerName,
                                                     file_sink,
                                                     spdlog::thread_pool(),
                                                     spdlog::async_overflow_policy::block);

        // We have some extra formatting on the log level %l below to keep color coding when dumping
        // json to the console and we use a full ISO 8601 time/date format
        std::string jsonPattern = {"{\"time\": \"%Y-%m-%dT%H:%M:%S.%f%z\", %v},"};

        // logger->set_pattern("[%H:%M:%S:%e] %v");
        logger->set_pattern(jsonPattern);
        spdlog::register_logger(logger);

        // Flush policy: https://github.com/gabime/spdlog/wiki/Flush-policy
        spdlog::flush_every(std::chrono::seconds(2));
    } catch (const spdlog::spdlog_ex &ex) {
        std::cout << "Log init failed: " << ex.what() << std::endl;
    }
}