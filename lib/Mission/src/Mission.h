#pragma once
// Máquina de estados de la misión y cola de primitivas.
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <vector>
#include "Planner.h"
#include "Motion.h"
#include "Pose.h"
#include "Imu.h"

class Brush;

class Mission {
 public:
  enum class State : uint8_t { INACTIVO, PLANEANDO, LISTO, EJECUTANDO, PAUSADO, COMPLETADO, FALLA };

  struct PlanRequest {
    planner::Cell start;
    uint8_t heading = planner::E;
    std::vector<planner::Cell> goals;
    planner::Algorithm algo = planner::Algorithm::ASTAR;
    planner::Heuristic heuristic = planner::Heuristic::MANHATTAN;
    bool returnHome = false;
  };

  struct Step {
    enum Kind : uint8_t { FORWARD, TURN, PWM } kind = FORWARD;
    float tx = 0, ty = 0, th = 0;
    float dutyL = 0, dutyR = 0;
    uint32_t ms = 0;
    planner::Cell target;
    uint8_t goalReached = planner::NO_GOAL;
    char label[24] = "";
  };

  void begin(Motion *m, Pose *p, Imu *imu, Brush *brush = nullptr);
  void setBrush(Brush *b) { brush_ = b; }

  void setObstacles(const std::vector<planner::Cell> &obs);
  planner::Grid grid() const;

  bool requestPlan(const PlanRequest &r, const char **err);
  void plannerLoop();
  void onPlanReady(void (*cb)()) { onPlanReady_ = cb; }

  void step();
  bool execute(bool confirmUnreachable, const char **err);
  bool togglePause(const char **err);
  void emergencyStop();
  void fail(const char *msg);
  bool runTest(const Step &s, const char **err);
  bool testForward(int cells, const char **err);
  bool testTurn(int degrees, const char **err);
  bool testPwm(float l, float r, uint32_t ms, const char **err);
  void setPoseAt(planner::Cell c, uint8_t heading);

  State state() const { return state_; }
  static const char *stateName(State s);
  const char *message() const { return message_; }
  size_t stepIndex() const { return idx_; }
  size_t stepCount() const;
  void currentLabel(char *out, size_t n) const;
  planner::Cell currentCell() const;
  uint32_t goalsReached() const { return goalsReached_; }

  template <class F>
  void withPlan(F f) const {
    xSemaphoreTake(lock_, portMAX_DELAY);
    f(planValid_, lastReq_, lastTour_);
    xSemaphoreGive(lock_);
  }

 private:
  void startStep();
  void finishStep();
  float continuousHeading(uint8_t h) const;
  float snapHeading(float th) const;
  void buildSteps();
  void saveObstacles();
  void loadObstacles();

  Motion *motion_ = nullptr;
  Pose *pose_ = nullptr;
  Imu *imu_ = nullptr;
  Brush *brush_ = nullptr;
  SemaphoreHandle_t lock_ = nullptr;
  TaskHandle_t plannerTask_ = nullptr;

  volatile State state_ = State::INACTIVO;
  const char *message_ = "";
  planner::Grid grid_;

  PlanRequest pendingReq_;
  PlanRequest lastReq_;
  planner::TourResult lastTour_;
  bool planValid_ = false;
  std::vector<Step> steps_;
  volatile size_t idx_ = 0;
  bool stepStarted_ = false;
  bool pauseRequested_ = false;
  uint32_t pauseT0_ = 0;
  bool inPause_ = false;
  bool testMode_ = false;
  uint32_t goalsReached_ = 0;
  bool poseValid_ = false;
  planner::Cell lastCell_;
  uint8_t lastHeading_ = planner::E;
  void (*onPlanReady_)() = nullptr;
};