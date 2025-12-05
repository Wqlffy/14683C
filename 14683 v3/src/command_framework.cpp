#include "command_framework.hpp"

#include <algorithm>

namespace robot {

void CommandScheduler::schedule(const std::shared_ptr<Command>& command) {
    if (!command) return;
    active_.push_back({command, false});
}

void CommandScheduler::run() {
    for (std::size_t i = 0; i < active_.size();) {
        auto& entry = active_[i];
        if (!entry.initialized) {
            entry.command->initialize();
            entry.initialized = true;
        }
        entry.command->execute();
        if (entry.command->isFinished()) {
            entry.command->end(false);
            active_.erase(active_.begin() + static_cast<long>(i));
        } else {
            ++i;
        }
    }
}

void CommandScheduler::cancelAll() {
    for (auto& entry : active_) entry.command->end(true);
    active_.clear();
}

SequentialCommandGroup::SequentialCommandGroup(std::vector<std::shared_ptr<Command>> commands)
    : commands_(std::move(commands)) {}

void SequentialCommandGroup::initialize() {
    current_ = 0;
    done_ = commands_.empty();
    currentInitialized_ = false;
}

void SequentialCommandGroup::execute() {
    if (done_ || current_ >= commands_.size()) {
        done_ = true;
        return;
    }

    auto& cmd = commands_[current_];
    if (!currentInitialized_) {
        cmd->initialize();
        currentInitialized_ = true;
    }
    cmd->execute();

    if (cmd->isFinished()) {
        cmd->end(false);
        current_++;
        currentInitialized_ = false;
        if (current_ >= commands_.size()) done_ = true;
    }
}

bool SequentialCommandGroup::isFinished() const {
    return done_;
}

void SequentialCommandGroup::end(bool interrupted) {
    if (!done_ && current_ < commands_.size()) {
        commands_[current_]->end(interrupted);
    }
}

ParallelRaceGroup::ParallelRaceGroup(std::vector<std::shared_ptr<Command>> commands)
    : commands_(std::move(commands)), initialized_(commands_.size(), false) {}

void ParallelRaceGroup::initialize() {
    finished_ = commands_.empty();
    std::fill(initialized_.begin(), initialized_.end(), false);
}

void ParallelRaceGroup::execute() {
    if (finished_) return;
    for (std::size_t i = 0; i < commands_.size(); ++i) {
        if (!initialized_[i]) {
            commands_[i]->initialize();
            initialized_[i] = true;
        }
        commands_[i]->execute();
        if (commands_[i]->isFinished()) {
            finished_ = true;
            for (std::size_t j = 0; j < commands_.size(); ++j) {
                if (j == i) {
                    commands_[j]->end(false);
                } else if (initialized_[j]) {
                    commands_[j]->end(true);
                }
            }
            break;
        }
    }
}

void ParallelRaceGroup::end(bool interrupted) {
    if (interrupted) {
        for (std::size_t i = 0; i < commands_.size(); ++i) {
            if (initialized_[i]) commands_[i]->end(true);
        }
    }
}

}  // namespace robot
