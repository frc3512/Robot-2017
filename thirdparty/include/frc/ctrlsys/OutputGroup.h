// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <vector>

#include <units/units.h>

#include "frc/Notifier.h"
#include "frc/ctrlsys/Output.h"

namespace frc {

/**
 * Allows grouping Output instances together to run in one thread.
 *
 * Each output's OutputFunc() is called at a regular interval. This can be used
 * to avoid unnecessary context switches for Output instances that are running
 * at the same sample rate and priority.
 */
class OutputGroup {
public:
    /**
     * Appends output to array.
     *
     * @param output the Output object to add to the array for round robin
     * @param outputs the other Output objects
     */
    template <class... Outputs>
    explicit OutputGroup(Output& output, Outputs&&... outputs)
      : OutputGroup(outputs...) {
      m_outputs.emplace_back(output);
    }

    explicit OutputGroup(Output& output);

    virtual ~OutputGroup() = default;

    void Enable(units::second_t period = INode::kDefaultPeriod);
    void Disable();

protected:
    virtual void OutputFunc();

private:
    std::vector<std::reference_wrapper<Output>> m_outputs;
    Notifier m_thread;
};

}  // namespace frc
