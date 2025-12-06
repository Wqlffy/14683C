#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include <cstdint>

#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace robot {

class Command {
  public:
    virtual ~Command() = default;
    virtual void initialize() {}
    virtual void execute() {}
    virtual bool isFinished() const { return true; }
    virtual void end(bool interrupted) { (void)interrupted; }
};

class CommandScheduler {
  public:
    void schedule(const std::shared_ptr<Command>& command);
    void run();
    void cancelAll();
    bool empty() const { return active_.empty(); }

  private:
    struct Entry {
        std::shared_ptr<Command> command;
        bool initialized{false};
    };

    std::vector<Entry> active_{};
};

class FunctionalCommand : public Command {
  public:
    FunctionalCommand(std::function<void()> init, std::function<void()> execute,
                      std::function<bool()> finished, std::function<void(bool)> end)
        : onInit_(std::move(init)),
          onExecute_(std::move(execute)),
          isFinished_(std::move(finished)),
          onEnd_(std::move(end)) {}

    void initialize() override {
        if (onInit_) onInit_();
    }

    void execute() override {
        if (onExecute_) onExecute_();
    }

    bool isFinished() const override {
        return isFinished_ ? isFinished_() : true;
    }

    void end(bool interrupted) override {
        if (onEnd_) onEnd_(interrupted);
    }

  private:
    std::function<void()> onInit_;
    std::function<void()> onExecute_;
    std::function<bool()> isFinished_;
    std::function<void(bool)> onEnd_;
};

class WaitCommand : public Command {
  public:
    explicit WaitCommand(uint32_t durationMs) : durationMs_(durationMs) {}

    void initialize() override { startMs_ = pros::millis(); }
    bool isFinished() const override { return (pros::millis() - startMs_) >= durationMs_; }

  private:
    uint32_t startMs_{0};
    uint32_t durationMs_;
};

class SequentialCommandGroup : public Command {
  public:
    explicit SequentialCommandGroup(std::vector<std::shared_ptr<Command>> commands);
    void initialize() override;
    void execute() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

  private:
    std::vector<std::shared_ptr<Command>> commands_;
    std::size_t current_{0};
    bool currentInitialized_{false};
    bool done_{false};
};

class ParallelRaceGroup : public Command {
  public:
    explicit ParallelRaceGroup(std::vector<std::shared_ptr<Command>> commands);
    void initialize() override;
    void execute() override;
    bool isFinished() const override { return finished_; }
    void end(bool interrupted) override;

  private:
    std::vector<std::shared_ptr<Command>> commands_;
    std::vector<bool> initialized_;
    bool finished_{false};
};

}
