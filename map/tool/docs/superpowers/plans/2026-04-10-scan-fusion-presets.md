# Scan Fusion Presets Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add scene presets and parameter overrides for scan fusion so live mapping can adapt to simulator and real-world obstacle density without hardcoded thresholds.

**Architecture:** Define one scan fusion config model and preset table, then route browser and desktop accumulation, filtering, and save paths through resolved effective parameters. Persist the chosen preset and effective values into saved map metadata.

**Tech Stack:** JavaScript, Python, existing browser client, existing desktop client, pytest

---

### Task 1: Define the shared scan fusion model

**Files:**
- Modify: `client/main.js`
- Modify: `client_desktop/app.py`
- Test: `tests/test_client_desktop_logic.py`

- [ ] **Step 1: Write failing tests for preset resolution and override merging**
- [ ] **Step 2: Run the targeted tests and confirm failure**
- [ ] **Step 3: Add preset tables and effective-config resolution helpers**
- [ ] **Step 4: Re-run the targeted tests and confirm pass**

### Task 2: Apply config to live accumulation and filtering

**Files:**
- Modify: `client/main.js`
- Modify: `client_desktop/app.py`
- Test: `tests/test_client_desktop_logic.py`

- [ ] **Step 1: Write failing tests for occupied filtering under `sim_clean` vs `indoor_sensitive`**
- [ ] **Step 2: Run the targeted tests and confirm failure**
- [ ] **Step 3: Replace hardcoded voxel, hit, free-ratio, and turn-skip literals with effective config**
- [ ] **Step 4: Re-run the targeted tests and confirm pass**

### Task 3: Persist config into saved map metadata

**Files:**
- Modify: `client/main.js`
- Modify: `client_desktop/app.py`
- Test: `tests/test_client_desktop_logic.py`
- Test: `tests/test_client_desktop_app_helpers.py`

- [ ] **Step 1: Write failing tests for saved bundle metadata and reload behavior**
- [ ] **Step 2: Run the targeted tests and confirm failure**
- [ ] **Step 3: Save selected preset and effective values into manifest metadata and restore them on load**
- [ ] **Step 4: Re-run the targeted tests and confirm pass**

### Task 4: Document and verify

**Files:**
- Modify: `docs/product_manual.md`
- Modify: `README.md`

- [ ] **Step 1: Document available presets and intended usage**
- [ ] **Step 2: Run the final targeted test suite**
- [ ] **Step 3: Summarize validation evidence**
