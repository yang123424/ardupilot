--[[
  脚本名: copter-set-ekf-origin.lua
  用途:   通过 RC 开关 / 参数触发，手动设置 EKF 原点（origin），
          解决光流定点（无 GPS 绝对位置）时无法在 Mission Planner 里设置
          EKF origin 的问题。

  参数表前缀: EKFO_  （在 MP 的"全部参数表"里搜索 EKFO 即可改）

  参数说明:
    EKFO_MODE    0 = 使用当前 GPS 位置作为原点
                 1 = 使用下面的预设坐标作为原点
    EKFO_LAT     预设纬度（十进制度，例如 35.3632640）
    EKFO_LNG     预设经度（十进制度，例如 149.1652352）
    EKFO_ALT     预设高度（米，AMSL 海拔）
    EKFO_FORCE   0 = 若 EKF 原点已设置则跳过（默认，安全）
                 1 = 强制重置原点（覆盖已设置的错误原点）
    EKFO_TRIG    软件触发位：从 0 改成 1 即触发一次（脚本执行后自动复位为 0）

  触发方式（二选一）:
    方式 A（推荐）: 将某个 RC 通道的 RCx_OPTION 设为 300 (Scripting1)，
                    拨到高位(HIGH)即触发一次。脚本做了边沿检测，不会重复触发。
    方式 B: 在 MP 参数表里把 EKFO_TRIG 从 0 改成 1，即触发一次。
            （脚本执行后会把 EKFO_TRIG 自动复位为 0，方便下次再触发。）

  使用方法:
    1. 把本脚本拷到 SD 卡的 APM/SCRIPTS/ 目录。
    2. 重启飞控，脚本自动加载，参数 EKFO_* 会出现在参数表里。
    3. 在 MP 里设好 EKFO_MODE / EKFO_LAT / EKFO_LNG / EKFO_ALT / EKFO_FORCE。
    4. 触发: 方式 A 拨 RC 开关到高位；或方式 B 把 EKFO_TRIG 改成 1。
    5. 看到 "EKF origin set ..." 提示即成功。
--]]

local PARAM_TABLE_KEY = 73
assert(param:add_table(PARAM_TABLE_KEY, "EKFO_", 6), 'could not add EKFO param table')

-- 参数索引 1..6，默认值如下

--[[
  // @Param: EKFO_MODE
  // @DisplayName: EKF Origin Source
  // @Description: 设置 EKF 原点的坐标来源。0=使用当前 GPS 位置，1=使用 EKFO_LAT/LNG/ALT 预设坐标
  // @Values: 0:GPS_Current,1:Preset_Coord
  // @User: Standard
--]]
assert(param:add_param(PARAM_TABLE_KEY, 1, 'MODE',   0), 'could not add EKFO_MODE')

--[[
  // @Param: EKFO_LAT
  // @DisplayName: Preset Origin Latitude
  // @Description: 预设 EKF 原点纬度（十进制度）
  // @Range: -90 90
  // @Units: deg
  // @User: Standard
--]]
assert(param:add_param(PARAM_TABLE_KEY, 2, 'LAT',    35.3632640), 'could not add EKFO_LAT')

--[[
  // @Param: EKFO_LNG
  // @DisplayName: Preset Origin Longitude
  // @Description: 预设 EKF 原点经度（十进制度）
  // @Range: -180 180
  // @Units: deg
  // @User: Standard
--]]
assert(param:add_param(PARAM_TABLE_KEY, 3, 'LNG',   149.1652352), 'could not add EKFO_LNG')

--[[
  // @Param: EKFO_ALT
  // @DisplayName: Preset Origin Altitude
  // @Description: 预设 EKF 原点高度（米，AMSL 海拔）
  // @Range: -500 10000
  // @Units: m
  // @User: Standard
--]]
assert(param:add_param(PARAM_TABLE_KEY, 4, 'ALT',   584.09), 'could not add EKFO_ALT')

--[[
  // @Param: EKFO_FORCE
  // @DisplayName: Force Origin Override
  // @Description: 0=若 EKF 原点已设置则跳过（安全）；1=强制覆盖已设置的原点
  // @Values: 0:No_Override,1:Force_Override
  // @User: Standard
--]]
assert(param:add_param(PARAM_TABLE_KEY, 5, 'FORCE',  0), 'could not add EKFO_FORCE')

--[[
  // @Param: EKFO_TRIG
  // @DisplayName: Software Trigger
  // @Description: 软件触发位：从 0 改成 1 即触发一次设置原点（脚本执行后自动复位为 0）
  // @Values: 0:Idle,1:Trigger
  // @User: Standard
--]]
assert(param:add_param(PARAM_TABLE_KEY, 6, 'TRIG',   0), 'could not add EKFO_TRIG')

local MODE_P  = Parameter("EKFO_MODE")
local LAT_P   = Parameter("EKFO_LAT")
local LNG_P   = Parameter("EKFO_LNG")
local ALT_P   = Parameter("EKFO_ALT")
local FORCE_P = Parameter("EKFO_FORCE")
local TRIG_P  = Parameter("EKFO_TRIG")

-- 用 RCx_OPTION = 300 (Scripting1) 作为触发开关
local rc_switch = rc:find_channel_for_option(300)

-- 触发用状态
local last_switch_pos = nil
local last_trig = nil

-- 把十进制度换算成 Location 的 1e-7 度整型
local function deg_to_1e7(deg)
    return math.floor(deg * 10000000 + 0.5)
end

-- 核心：执行一次设置 EKF 原点
local function do_set_origin()
    -- 1) 若未允许强制覆盖，且原点已存在，则跳过
    local force = FORCE_P:get()
    if force < 0.5 then
        local existing = ahrs:get_origin()
        if existing then
            gcs:send_text(6, string.format("EKF origin already set, skipped (set EKFO_FORCE=1 to override)"))
            return
        end
    end

    -- 2) 决定坐标来源
    local loc = nil
    local mode = MODE_P:get()

    if mode < 0.5 then
        -- 使用当前 GPS 位置
        if gps:num_sensors() > 0 and gps:status(0) >= gps.GPS_OK_FIX_3D then
            loc = gps:location(0)
            gcs:send_text(6, "Using GPS fix as EKF origin")
        else
            gcs:send_text(3, "No 3D GPS fix, cannot use GPS origin. Switch EKFO_MODE=1 or fix GPS.")
            return
        end
    else
        -- 使用预设坐标
        loc = Location()
        loc:lat(deg_to_1e7(LAT_P:get()))
        loc:lng(deg_to_1e7(LNG_P:get()))
        loc:alt(math.floor(ALT_P:get() * 100 + 0.5)) -- 米 -> 厘米
    end

    -- 3) 写入 EKF origin
    if ahrs:set_origin(loc) then
        gcs:send_text(6, string.format("EKF origin set - Lat:%.7f Lon:%.7f Alt:%.1f",
            loc:lat() / 10000000, loc:lng() / 10000000, loc:alt() / 100))
    else
        gcs:send_text(3, "Failed to set EKF origin")
    end
end

function update()
    local triggered = false

    -- 触发方式 A: RC 开关 (RCx_OPTION=300) 拨到高位，上升沿触发一次
    if rc_switch then
        local pos = rc_switch:get_aux_switch_pos()
        if pos == 2 and last_switch_pos ~= 2 then
            triggered = true
        end
        last_switch_pos = pos
    end

    -- 触发方式 B: EKFO_TRIG 从 0 改成 1，触发一次并自动复位
    local trig = TRIG_P:get()
    if last_trig == nil then
        last_trig = trig
    elseif trig ~= last_trig then
        if trig >= 0.5 then
            triggered = true
            TRIG_P:set(0)        -- 自动复位
            last_trig = 0        -- 同步本地状态，避免误触发
        else
            last_trig = trig
        end
    end

    if triggered then
        do_set_origin()
    end

    return update, 200
end

gcs:send_text(6, "copter-set-ekf-origin loaded. Trigger: RCx_OPTION=300 HIGH, or EKFO_TRIG=1")
return update()
