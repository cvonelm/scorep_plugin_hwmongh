/*
 * Copyright (c) 2016, Technische Universität Dresden, Germany
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cassert>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <thread>
#include <fstream>
#include <vector>

#include <scorep/plugin/plugin.hpp>

class GHSensor
{
    public:
    GHSensor(std::string metric_name) : metric_name_(metric_name)
    {
    }
    std::string metric_name() const
    {
        return metric_name_;
    }

    friend bool operator<(const GHSensor& lhs, const GHSensor&rhs)
    {
        return lhs.metric_name_ < rhs.metric_name_;
    }
private:
    std::string metric_name_;
};
using TVPair = std::pair<scorep::chrono::ticks, double>;

static const std::string hwmon_path = "/sys/class/hwmon";
using scorep::plugin::logging;
class GHMeasurementThread
{
public:
    void measurement()
    {
        while (!stop_)
        {
            for (auto&sensor : sensors_)
            {
                uint64_t val;
                sensor.second >> val;
                sensor.second.seekg(0, std::ifstream::beg);
                if(val != 0)
                {
                    data_[sensor.first].emplace_back(scorep::chrono::measurement_clock::now(), val/1000000);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void stop_measurement()
    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        stop_ = true;
    }

    void add_sensor(GHSensor sensor)
    {
        data_[sensor] = std::vector<TVPair>();
        std::string path = hwmon_path + "/" + sensor.metric_name() + "/device/power1_average";
        std::cout << path << std::endl;
        sensors_[sensor] = std::ifstream(path);
    }

    std::vector<TVPair> get_values(GHSensor sensor)
    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        return data_[sensor];
    }

private:
    std::map<GHSensor, std::vector<TVPair>> data_;
    bool stop_ = false;
    std::mutex read_mutex_;
    std::map<GHSensor, std::ifstream> sensors_;
};

using namespace scorep::plugin::policy;

template <typename T, typename Policies>
using hwmongh_object_id = object_id<GHSensor, T, Policies>;

class hwmongh_plugin : public scorep::plugin::base<hwmongh_plugin, async, once, scorep_clock, hwmongh_object_id>
{
public:
    hwmongh_plugin()
    {
    }

    ~hwmongh_plugin()
    {
    }

    std::vector<scorep::plugin::metric_property>
    get_metric_properties(const std::string& metric_name)
    {

        std::ifstream sensor_name  = std::ifstream(hwmon_path + "/" + metric_name + "/device/power1_oem_info");

        if(!sensor_name.good())
        {
            throw std::runtime_error("Unknwon sensor: " + metric_name);
        }

        std::string name;
        sensor_name >> name;
        GHSensor sensor(metric_name);
        hwmongh.add_sensor(sensor);
        std::vector<scorep::plugin::metric_property> sensors;


                    sensors.push_back(scorep::plugin::metric_property(
                                          metric_name, name, "W")
                                          .absolute_point()
                                          .value_double());

                    make_handle(metric_name, sensor);
                    return sensors;
    }

    void add_metric(GHSensor& f)
    {
    }

    template <typename C>
    void get_all_values(GHSensor& f, C& cursor)
    {
        std::vector<TVPair> values = hwmongh.get_values(f);

        for (auto& value : values)
        {
            cursor.write(value.first, value.second);
        }
    }

    void start()
    {
        measurement_thread_ = std::thread([this]() { this->hwmongh.measurement(); });
    }

    void stop()
    {
        hwmongh.stop_measurement();

        if (measurement_thread_.joinable())

        {
            measurement_thread_.join();
        }
    }

private:

    GHMeasurementThread hwmongh;
    std::thread measurement_thread_;
};

SCOREP_METRIC_PLUGIN_CLASS(hwmongh_plugin, "hwmongh")
