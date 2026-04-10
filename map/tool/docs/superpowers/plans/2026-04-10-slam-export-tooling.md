# SLAM Export Tooling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build Java and C++ utility libraries that parse `.slam` archives and generate `.pgm`, `.yaml`, and `.json` exports in memory and on disk.

**Architecture:** Add an isolated `tools/slam_export` area with one Java module and one C++ module. Both modules implement the same archive parsing and export rules described in the spec so they can be compiled and tested independently.

**Tech Stack:** Java 11, Maven, Jackson, JUnit 5, C++17, CMake, minizip, nlohmann/json, CTest

---

### Task 1: Add shared documentation

**Files:**
- Create: `docs/superpowers/specs/2026-04-10-slam-export-tooling-design.md`
- Create: `docs/superpowers/plans/2026-04-10-slam-export-tooling.md`
- Modify: `README.md`

- [ ] **Step 1: Write the design doc**
- [ ] **Step 2: Add the implementation plan**
- [ ] **Step 3: Document where the new tooling lives**

### Task 2: Add Java module and tests

**Files:**
- Create: `tools/slam_export/java/pom.xml`
- Create: `tools/slam_export/java/src/main/java/com/autodrive/slamexport/*.java`
- Create: `tools/slam_export/java/src/test/java/com/autodrive/slamexport/*.java`

- [ ] **Step 1: Write failing Java tests for load and export behavior**
- [ ] **Step 2: Run Maven tests and confirm failure**
- [ ] **Step 3: Implement archive parsing and export generation**
- [ ] **Step 4: Re-run Maven tests and confirm pass**

### Task 3: Add C++ module and tests

**Files:**
- Create: `tools/slam_export/cpp/CMakeLists.txt`
- Create: `tools/slam_export/cpp/include/slam_export/*.hpp`
- Create: `tools/slam_export/cpp/src/*.cpp`
- Create: `tools/slam_export/cpp/tests/*.cpp`

- [ ] **Step 1: Write failing C++ tests for load and export behavior**
- [ ] **Step 2: Configure and run CTest to confirm failure**
- [ ] **Step 3: Implement archive parsing and export generation**
- [ ] **Step 4: Re-run CTest and confirm pass**

### Task 4: Verify and summarize

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Run Java verification**
- [ ] **Step 2: Run C++ verification**
- [ ] **Step 3: Update README with final module paths and commands**
