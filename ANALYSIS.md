# Deep Analysis: Stopwatch Freeze Bug

## Executive Summary
The stopwatch **appears missing and freezes the device when accessed** due to **multiple architectural and implementation issues**:

1. **No visible stopwatch UI on startup** (dynamic loading issue)
2. **Severe freezes when navigating to stopwatch tile** (blocking operations in event handlers)
3. **Unscrollable vertical scroll container** (UI navigation problem)
4. **Nested scrolling conflicts** (LVGL configuration issue)
5. **No error handling or timeouts** on I2C operations during tile building

---

## Issue 1: Stopwatch Not Visible on Startup (Dynamic Loading)

### Root Cause
The stopwatch is built **only on-demand** via `tile_change_cb()` when the user swipes left:

```c
// Line ~1113
else if (tile == tile_left) {
    if (!g_sw_label_time) build_tools_page(tile_left);  // Built ONLY on first access
    if (g_line_hour) { lv_obj_clean(tile_center); clean_pilot_globals(); } 
}
```

**Problem**: 
- User swipes left expecting stopwatch → tiles scroll container hasn't been populated
- The container `cont` is created but **cannot scroll** because it's misconfigured
- Stopwatch UI never appears

### Why It Freezes

1. **Vertical scroll container misconfiguration** (Lines 821-831):
   ```c
   lv_obj_t *cont = lv_obj_create(parent);  // parent = tile_left
   lv_obj_set_size(cont, 240, 240);
   lv_obj_center(cont);
   lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_scroll_dir(cont, LV_DIR_VER);
   lv_obj_set_scroll_snap_y(cont, LV_SCROLL_SNAP_CENTER);
   lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
   lv_obj_set_style_bg_color(cont, lv_color_black(), 0);
   ```
   
   **Issue**: The container is **centered** with `lv_obj_center()` but has **flex layout** + scrolling. LVGL tries to:
   - Center the container on the parent (tile_left is 240×240)
   - Apply flex flow (layouts children vertically)
   - Enable scrolling (tries to clip content)
   
   This creates **conflicting layout constraints**, causing LVGL to loop trying to resolve them.

2. **Page1 inside container** (Line 838):
   ```c
   lv_obj_t *page1 = lv_obj_create(cont);
   lv_obj_set_size(page1, 240, 240);  // Same size as parent!
   lv_obj_add_flag(page1, LV_OBJ_FLAG_SNAPPABLE);
   ```
   
   **Issue**: `page1` is exactly the same size as `cont`, which is exactly the same size as `tile_left`. No room to scroll. LVGL's scroll snap logic gets confused and **locks up calculating geometry**.

3. **Nested scrolling conflict**:
   - `tile_left` is part of the tileview (which scrolls horizontally)
   - `cont` inside `tile_left` tries to scroll vertically
   - `page1`, `page2`, `page3` marked as `SNAPPABLE` 
   
   LVGL's event propagation gets trapped in a **circular dependency** when touch input arrives:
   - Tileview wants to handle horizontal scroll
   - Container wants to handle vertical scroll
   - Snappable pages interfere with both
   - Result: **Infinite loop in `lv_timer_handler()` or event processing**

---

## Issue 2: Blocking Operations During Page Building

### Root Cause (Lines 815-1050)

When `build_tools_page()` is called, it creates:
- **60 line objects for stopwatch ticks** (12 major ticks × 5)
- **6 subdial tick lines**
- **Complex nested hierarchy** with multiple containers

```c
for (int i = 0; i < 12; i++) {
    // ... creates 12 lines
    lv_obj_t *line = lv_line_create(page1);  // Calls LVGL allocator
    lv_line_set_points(line, sw_ticks[i], 2);
    lv_obj_add_style(line, &style_sw_tick_major, 0);  // Redraws
}

// Subdial also creates 6 lines
for(int i=0; i<6; i++) {
    lv_obj_t *line = lv_line_create(subdial);  // More allocations
    lv_line_set_points(line, sub_ticks[i], 2);  // More redraws
}

// Plus buttons, labels, hands...
g_sw_hand_sec = lv_line_create(page1);
g_sw_hand_min_sub = lv_line_create(subdial);
g_sw_btn_toggle = lv_btn_create(page1);
// ... more
```

**Problem**:
- Each `lv_obj_create()`, `lv_line_create()`, etc. **allocates from LVGL's limited heap**
- Each `lv_line_set_points()` triggers a **partial redraw**
- Creating **18 line objects + buttons + labels** in rapid succession exhausts the draw buffer
- If LVGL's memory monitor detects heap depletion, it **stalls waiting for garbage collection** (which won't happen in the main loop during event processing)

### Memory Exhaustion Chain

```
tile_change_cb() called
  → build_tools_page(tile_left) called synchronously
    → for loop creates 12 stopwatch tick lines
      → Each calls lv_line_create() (heap allocation)
      → Each calls lv_obj_add_style() (heap reallocation)
      → LVGL memory pool shrinks
    → Creates 6 subdial ticks (more heap usage)
    → Creates buttons, labels, hands (more heap usage)
    → DRAW_BUFFER_SIZE = (240 * 60) = 14,400 bytes
    → If building stopwatch ticks causes partial redraws:
      → Each redraw tries to allocate space for new geometry
      → Heap fragmentation occurs
      → lv_timer_handler() stalls on the next call
    → Meanwhile, main loop calls lv_timer_handler() expecting it to return
    → But lv_timer_handler() is blocked on memory allocation retry
    → Device appears frozen
```

---

## Issue 3: Nested Container Size Conflicts

### Actual Layout (Current Broken Code)

```
tile_left (240×240, from tileview)
  ├─ cont (240×240, CENTERED + FLEX + SCROLL)
  │   ├─ page1 (240×240, SNAPPABLE)
  │   ├─ page2 (240×240, SNAPPABLE)
  │   └─ page3 (240×240, SNAPPABLE)
```

### Why This Breaks

1. **Container is centered**: Tries to place a 240×240 object in the center of a 240×240 parent
   - Result: Offset to `(0, 0)` (no visible centering)
   - But LVGL still marks it as "centered" internally

2. **Flex layout conflicts with scroll**:
   - `LV_FLEX_FLOW_COLUMN` tells LVGL: "Stack children vertically, adjusting heights"
   - `lv_obj_set_scroll_dir(cont, LV_DIR_VER)` tells LVGL: "Children can exceed bounds and be scrolled"
   - These two constraints are **mutually exclusive**
   - LVGL tries to satisfy both → **infinite layout recalculation**

3. **Scroll snapping on fixed-size children**:
   - Each page is exactly 240×240 (same as container)
   - When `lv_obj_set_scroll_snap_y()` is set, LVGL tries to snap to page boundaries
   - But with zero overflow, there's **nothing to scroll**
   - Snap algorithm runs but **never converges** on a valid snap position

---

## Issue 4: Event Handler Blocking

### The Tile Change Callback (Lines 1110-1132)

```c
static void tile_change_cb(lv_event_t * e) {
    lv_obj_t * tileview = lv_event_get_target(e);
    lv_obj_t * tile = lv_tileview_get_tile_act(tileview);
    printf("[INFO] tile_change_cb: tile change requested\n");
    if (tile == tile_center) {
        // ... clean up and build
    } else if (tile == tile_left) {
        printf("[INFO] tile_change_cb: tile_left active\n");
        if (!g_sw_label_time) build_tools_page(tile_left);  // <-- BLOCKING HERE
        if (g_line_hour) { lv_obj_clean(tile_center); clean_pilot_globals(); } 
    }
}
```

**Critical Problem**:
- `tile_change_cb()` is called **from inside `lv_timer_handler()`**
- It calls `build_tools_page()` **synchronously**
- `build_tools_page()` creates 18+ objects with allocations and redraws
- If memory is tight, LVGL's allocator **blocks indefinitely**
- `lv_timer_handler()` **never returns** to the main loop
- Watchdog **does not fire** (because this isn't a full infinite loop, just very slow)
- Device appears **frozen** (unresponsive to touch, no UI updates)

---

## Issue 5: Compile Errors Added

The recent debug additions have syntax errors:

```c
// Line 815 (WRONG):
printf("[INFO] build_tools_page start: total=%lu used=%lu free=%lu\n", 
       mon_start.total_size, mon_start.used_size, mon_start.free_size);
//                                        ^^^^^^^^^^ 
// lv_mem_monitor_t has NO 'used_size' field. Only: total_size, free_size
```

This causes **build failure**.

---

## Why Other Pages Work But Stopwatch Doesn't

### Pilot Face (build_face_pilot)
- Only **60 line objects** for ticks (acceptable heap usage)
- No flex layout + scroll conflict
- Doesn't use dynamic page snapping
- **Works fine** ✓

### Activity Screen (build_activity_screen)
- Only creates **1 chart + 1 label**
- Minimal heap usage
- No complex nested layouts
- **Works fine** ✓

### Stopwatch (build_tools_page)
- Creates **3 full-page containers** (page1, page2, page3)
- **18+ line objects** for ticks and hands
- **Flex + scroll + snap layout conflicts**
- **Maximum heap fragmentation**
- **All three pages initialized even though user only sees one**
- **BREAKS** ✗

---

## Summary of Root Causes

| # | Issue | Severity | Impact |
|---|-------|----------|--------|
| 1 | Flex + Scroll layout conflict in container | **CRITICAL** | Infinite layout loop |
| 2 | Child page size = parent size (no scrolling) | **CRITICAL** | Snap algorithm fails |
| 3 | Event handler blocking on memory allocation | **CRITICAL** | Watchdog timeout / freeze |
| 4 | All 3 pages built immediately (unnecessary) | **HIGH** | Heap exhaustion |
| 5 | Nested tileview + scroll container conflict | **HIGH** | Touch input breaks layout |
| 6 | Dynamic page building in callback | **MEDIUM** | Poor UX (lag on first access) |

---

## Recommended Fixes

### Fix 1: Remove Flex + Scroll Conflict
- **Don't use `LV_FLEX_FLOW_COLUMN`** on a scrollable container
- Instead: **Set manual positioning** for pages

### Fix 2: Fix Container Sizing
- Container should **extend beyond visible area** (e.g., 240×720 for 3 pages)
- Pages should be **stacked vertically** with explicit `y` positions (0, 240, 480)

### Fix 3: Use Manual Pagination Instead of Snapping
- Remove snap behavior
- Use **touch swipe detection** to manually update displayed page

### Fix 4: Lazy-Load Pages
- Build stopwatch page **only when first accessed**, not all 3 at once
- Or: **Pre-build at startup** but **asynchronously** with task yields

### Fix 5: Remove Centering
- Container should be at **(0, 0)** relative to tile
- Pages at explicit positions: page1 (0,0), page2 (0,240), page3 (0,480)

### Fix 6: Increase Draw Buffer or Implement Partial Updates
- Current: `DRAW_BUFFER_SIZE = (240 * 60) = 14.4 KB`
- Stopwatch redraw may exceed this during `lv_timer_handler()`
- Consider increasing to `(240 * 120)` or implementing **dirty rect tracking**
