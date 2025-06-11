#pragma once

#include <mutex>
#include <spdlog/spdlog.h>
#include <toml.hpp>

struct CoreConfigType
{
    // config entries
    bool enableForceSensor{false};
    std::string usWindowDisplayName{};
    std::string shellSensorPortName{};
};

/**
 * The Singleton class defines the `GetInstance` method that serves as an
 * alternative to constructor and lets clients access the same instance of this
 * class over and over.
 */
class ConfigInstance
{
    /**
     * The Singleton's constructor/destructor should always be private to
     * prevent direct construction/desctruction calls with the `new`/`delete`
     * operator.
     */
    static ConfigInstance *pConfiginstance_;
    static std::mutex configMutex_;

    // config entries
    CoreConfigType coreConfig_;

protected:
    ConfigInstance()
    {
        spdlog::info("Loading config file and initializing main config...");
        // select TOML version at runtime (optional)
        try {
            std::string coreConfigPath = "configs/core.toml";
            const auto data = toml::parse(coreConfigPath, toml::spec::v(1, 1, 0));
            spdlog::info(
                "Loading " + coreConfigPath + ": " + toml::find_or<std::string>(
                    data,
                    "description",
                    "not found"));

            // update main config entries
            coreConfig_.enableForceSensor = toml::find_or<bool>(data, "enableForceSensor", false);

            coreConfig_.usWindowDisplayName = toml::find_or<std::string>(
                data,
                "USWindowDisplayName",
                "");

            coreConfig_.shellSensorPortName = toml::find_or<std::string>(
                data,
                "shellSensorPortName",
                "");
        } catch (const toml::exception &err) {
            spdlog::error("Error parsing config file: {}", err.what());
        }
    }

public:
    /**
     * Singletons should not be cloneable.
     */
    ConfigInstance(ConfigInstance &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const ConfigInstance &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */

    static ConfigInstance *GetInstance();

    [[nodiscard]] CoreConfigType core() const
    {
        return coreConfig_;
    }
};

/**
 * Static methods should be defined outside the class.
 */

inline ConfigInstance *ConfigInstance::pConfiginstance_{nullptr};
inline std::mutex ConfigInstance::configMutex_;

/**
 * The first time we call GetInstance we will lock the storage location
 *      and then we make sure again that the variable is null and then we
 *      set the value. RU:
 */
inline ConfigInstance *ConfigInstance::GetInstance()
{
    std::lock_guard<std::mutex> lock(configMutex_);
    if (pConfiginstance_ == nullptr) {
        pConfiginstance_ = new ConfigInstance();
    }
    return pConfiginstance_;
}

// global variable for accessing configs
inline const auto &gCoreConfig = ConfigInstance::GetInstance()->core();