-- -*- mode: Lua; indent-tabs-mode: nil; lua-indent-level: 4; -*-

--------------------------------------------------------------------------------
-- CoppeliaSim Callbacks
--------------------------------------------------------------------------------

function sysCall_init()
    g_joint_x = sim.getObject('./RobotJointX')
    g_joint_y = sim.getObject('./RobotJointY')
    g_joint_z = sim.getObject('./RobotJointZ')

    g_drive            = sim.getObject('/Drive')
    g_gripper          = sim.getObject('/Gripper')
    g_proximity_sensor = sim.getObject('./ProximitySensor')

    g_state = {
        state  = STATE_HALT_UNLOADED,
        target = nil
    }

    ui = simUI.create(UI_XML)
end


function sysCall_actuation()
    local old_state = g_state.state

    state_transition()

    if old_state ~= g_state.state then
        log_info("State transition: " .. old_state .. " -> " .. g_state.state)
    end
end


--------------------------------------------------------------------------------
-- Finite State Machine States
--------------------------------------------------------------------------------

STATE_HALT_UNLOADED                     =  0

STATE_PICKING_PREPARING                 =  1
STATE_PICKING_ISSUING_MOVE_X            =  2
STATE_PICKING_MOVING_X                  =  3
STATE_PICKING_ISSUING_MOVE_Z            =  4
STATE_PICKING_MOVING_Z                  =  5
STATE_PICKING_ISSUING_MOVE_Y_FORWARD    =  6
STATE_PICKING_MOVING_Y_FORWARD          =  7
STATE_PICKING_ATTACHING                 =  8
STATE_PICKING_ISSUING_MOVE_Y_BACKWARD   =  9
STATE_PICKING_MOVING_Y_BACKWARD         = 10

STATE_LOADING_PREPARING                 = 11
STATE_LOADING_ISSUING_MOVE_X            = 12
STATE_LOADING_MOVING_X                  = 13
STATE_LOADING_ISSUING_MOVE_Z            = 14
STATE_LOADING_MOVING_Z                  = 15
STATE_LOADING_ISSUING_MOVE_Y_FORWARD    = 16
STATE_LOADING_MOVING_Y_FORWARD          = 17
STATE_LOADING_ATTACHING                 = 18
STATE_LOADING_ISSUING_MOVE_Y_BACKWARD   = 19
STATE_LOADING_MOVING_Y_BACKWARD         = 20

STATE_HALT_LOADED                       = 21

STATE_UNLOADING_PREPARING               = 22
STATE_UNLOADING_ISSUING_MOVE_Y_FORWARD  = 23
STATE_UNLOADING_MOVING_Y_FORWARD        = 24
STATE_UNLOADING_ATTACHING               = 25
STATE_UNLOADING_ISSUING_MOVE_Y_BACKWARD = 26
STATE_UNLOADING_MOVING_Y_BACKWARD       = 27

STATE_STORING_PREPARING                 = 28
STATE_STORING_ISSUING_MOVE_X            = 29
STATE_STORING_MOVING_X                  = 30
STATE_STORING_ISSUING_MOVE_Z            = 31
STATE_STORING_MOVING_Z                  = 32
STATE_STORING_ISSUING_MOVE_Y_FORWARD    = 33
STATE_STORING_MOVING_Y_FORWARD          = 34
STATE_STORING_ATTACHING                 = 35
STATE_STORING_ISSUING_MOVE_Y_BACKWARD   = 36
STATE_STORING_MOVING_Y_BACKWARD         = 37


--------------------------------------------------------------------------------
-- Finite State Machine Transitions
--------------------------------------------------------------------------------

JOINT_X_HOME_POSITION = 0.0
JOINT_Y_HOME_POSITION = 0.0
JOINT_Z_HOME_POSITION = 0.0

DRIVE_OFFSET_BASE_Y   = 0.10

function state_transition()
    if g_state.state == STATE_HALT_UNLOADED then
        return
    end

    if g_state.state == STATE_PICKING_PREPARING then
        g_state.state = STATE_PICKING_ISSUING_MOVE_X
        return
    end

    if g_state.state == STATE_PICKING_ISSUING_MOVE_X then
        move_joint(g_joint_x, g_state.target.x)
        g_state.state = STATE_PICKING_MOVING_X
        return
    end

    if g_state.state == STATE_PICKING_MOVING_X then
        if has_joint_reached_target(g_joint_x, g_state.target.x) then
            g_state.state = STATE_PICKING_ISSUING_MOVE_Z
        end
        return
    end

    if g_state.state == STATE_PICKING_ISSUING_MOVE_Z then
        move_joint(g_joint_z, g_state.target.z)
        g_state.state = STATE_PICKING_MOVING_Z
        return
    end

    if g_state.state == STATE_PICKING_MOVING_Z then
        if has_joint_reached_target(g_joint_z, g_state.target.z) then
            g_state.state = STATE_PICKING_ISSUING_MOVE_Y_FORWARD
        end
        return
    end

    if g_state.state == STATE_PICKING_ISSUING_MOVE_Y_FORWARD then
        move_joint(g_joint_y, g_state.target.y)
        g_state.state = STATE_PICKING_MOVING_Y_FORWARD
        return
    end

    if g_state.state == STATE_PICKING_MOVING_Y_FORWARD then
        if has_joint_reached_target(g_joint_y, g_state.target.y) then
            g_state.state = STATE_PICKING_ATTACHING
        end
        return
    end

    if g_state.state == STATE_PICKING_ATTACHING then
        local target = get_proximity_sensor_target(g_proximity_sensor)
        if target ~= nil then
            detach_object(target)
            -- sim.setInt32Signal('gripper', 1)
            attach_objects(g_gripper, target)
            g_state.state = STATE_PICKING_ISSUING_MOVE_Y_BACKWARD
        end
        return
    end

    if g_state.state == STATE_PICKING_ISSUING_MOVE_Y_BACKWARD then
        move_joint(g_joint_y, JOINT_Y_HOME_POSITION)
        g_state.state = STATE_PICKING_MOVING_Y_BACKWARD
        return
    end

    if g_state.state == STATE_PICKING_MOVING_Y_BACKWARD then
        if has_joint_reached_target(g_joint_y, JOINT_Y_HOME_POSITION) then
            g_state.state = STATE_LOADING_PREPARING
        end
        return
    end

    if g_state.state == STATE_LOADING_PREPARING then
        g_state.state = STATE_LOADING_ISSUING_MOVE_X
        return
    end

    if g_state.state == STATE_LOADING_ISSUING_MOVE_X then
        move_joint(g_joint_x, JOINT_X_HOME_POSITION)
        g_state.state = STATE_LOADING_MOVING_X
        return
    end

    if g_state.state == STATE_LOADING_MOVING_X then
        if has_joint_reached_target(g_joint_x, JOINT_X_HOME_POSITION) then
            g_state.state = STATE_LOADING_ISSUING_MOVE_Z
        end
        return
    end

    if g_state.state == STATE_LOADING_ISSUING_MOVE_Z then
        move_joint(g_joint_z, JOINT_Z_HOME_POSITION)
        g_state.state = STATE_LOADING_MOVING_Z
        return
    end

    if g_state.state == STATE_LOADING_MOVING_Z then
        if has_joint_reached_target(g_joint_z, JOINT_Z_HOME_POSITION) then
            g_state.state = STATE_LOADING_ISSUING_MOVE_Y_FORWARD
        end
        return
    end

    if g_state.state == STATE_LOADING_ISSUING_MOVE_Y_FORWARD then
        move_joint(g_joint_y, DRIVE_OFFSET_BASE_Y)
        g_state.state = STATE_LOADING_MOVING_Y_FORWARD
        return
    end

    if g_state.state == STATE_LOADING_MOVING_Y_FORWARD then
        if has_joint_reached_target(g_joint_y, DRIVE_OFFSET_BASE_Y) then
            g_state.state = STATE_LOADING_ATTACHING
        end
        return
    end

    if g_state.state == STATE_LOADING_ATTACHING then
        local target = get_proximity_sensor_target(g_proximity_sensor)
        if target ~= nil then
            detach_object(target)
            attach_objects(g_drive, target)
            -- sim.setInt32Signal('gripper', 0)
            g_state.state = STATE_LOADING_ISSUING_MOVE_Y_BACKWARD
        end
        return
    end

    if g_state.state == STATE_LOADING_ISSUING_MOVE_Y_BACKWARD then
        move_joint(g_joint_y, JOINT_Y_HOME_POSITION)
        g_state.state = STATE_LOADING_MOVING_Y_BACKWARD
        return
    end

    if g_state.state == STATE_LOADING_MOVING_Y_BACKWARD then
        if has_joint_reached_target(g_joint_y, JOINT_Y_HOME_POSITION) then
            g_state.state = STATE_HALT_LOADED
        end
        return
    end

    if g_state.state == STATE_HALT_LOADED then
        return
    end

    if g_state.state == STATE_UNLOADING_PREPARING then
        g_state.state = STATE_UNLOADING_ISSUING_MOVE_Y_FORWARD
    end

    if g_state.state == STATE_UNLOADING_ISSUING_MOVE_Y_FORWARD then
        move_joint(g_joint_y, DRIVE_OFFSET_BASE_Y)
        g_state.state = STATE_UNLOADING_MOVING_Y_FORWARD
        return
    end

    if g_state.state == STATE_UNLOADING_MOVING_Y_FORWARD then
        if has_joint_reached_target(g_joint_y, DRIVE_OFFSET_BASE_Y) then
            g_state.state = STATE_UNLOADING_ATTACHING
        end
        return
    end

    if g_state.state == STATE_UNLOADING_ATTACHING then
        local target = get_proximity_sensor_target(g_proximity_sensor)
        if target ~= nil then
            detach_object(target)
            attach_objects(g_gripper, target)
            g_state.state = STATE_UNLOADING_ISSUING_MOVE_Y_BACKWARD
        end
        return
    end 

    if g_state.state == STATE_UNLOADING_ISSUING_MOVE_Y_BACKWARD then
        move_joint(g_joint_y, JOINT_Y_HOME_POSITION)
        g_state.state = STATE_UNLOADING_MOVING_Y_BACKWARD
    end

    if g_state.state == STATE_UNLOADING_MOVING_Y_BACKWARD then
        if has_joint_reached_target(g_joint_y, JOINT_Y_HOME_POSITION) then
            g_state.state = STATE_STORING_PREPARING
        end
        return
    end

    if g_state.state == STATE_STORING_PREPARING then
        g_state.state = STATE_STORING_ISSUING_MOVE_X
	return
    end

    if g_state.state == STATE_STORING_ISSUING_MOVE_X then
	move_joint(g_joint_x, g_state.target.x)
        g_state.state = STATE_STORING_MOVING_X
        return
    end

    if g_state.state == STATE_STORING_MOVING_X then
        if has_joint_reached_target(g_joint_x, g_state.target.x) then
            g_state.state = STATE_STORING_ISSUING_MOVE_Z
        end
        return
    end

    if g_state.state == STATE_STORING_ISSUING_MOVE_Z then
        move_joint(g_joint_z, g_state.target.z)
        g_state.state = STATE_STORING_MOVING_Z
        return
    end

    if g_state.state == STATE_STORING_MOVING_Z then
        if has_joint_reached_target(g_joint_z, g_state.target.z) then
            g_state.state = STATE_STORING_ISSUING_MOVE_Y_FORWARD
        end
        return
    end

    if g_state.state == STATE_STORING_ISSUING_MOVE_Y_FORWARD then
        move_joint(g_joint_y, g_state.target.y)
        g_state.state = STATE_STORING_MOVING_Y_FORWARD
        return
    end

    if g_state.state == STATE_STORING_MOVING_Y_FORWARD then
        if has_joint_reached_target(g_joint_y, g_state.target.y) then
            g_state.state = STATE_STORING_ATTACHING
        end
        return
    end

    if g_state.state == STATE_STORING_ATTACHING then
        local target = get_proximity_sensor_target(g_proximity_sensor)
        if target ~= nil then
	    -- TODO: attach to shelf
            detach_object(target)
            g_state.state = STATE_STORING_ISSUING_MOVE_Y_BACKWARD
        end
        return
    end

    if g_state.state == STATE_STORING_ISSUING_MOVE_Y_BACKWARD then
        move_joint(g_joint_y, JOINT_Y_HOME_POSITION)
        g_state.state = STATE_STORING_MOVING_Y_BACKWARD
        return
    end

    if g_state.state == STATE_STORING_MOVING_Y_BACKWARD then
        if has_joint_reached_target(g_joint_y, JOINT_Y_HOME_POSITION) then
            g_state.state = STATE_HALT_UNLOADED
        end
        return
    end

    fatal("State transition handler reached unreacheable control flow.")
end


--------------------------------------------------------------------------------
-- Cartridge Helpers
--------------------------------------------------------------------------------

OFFSET_BASE_X  = 0.42
OFFSET_DELTA_X = 0.05 + 0.02155

OFFSET_Y = 0.15

OFFSET_BASE_Z  = 0.00
OFFSET_DELTA_Z = 0.15

function slot_position(shelf, slot)
    -- first shelf = 0, last = 31
    -- first slot  = 0, last =  7
    return {
        x = OFFSET_BASE_X + slot * OFFSET_DELTA_X,
        y = OFFSET_Y,
        z = OFFSET_BASE_Z + shelf * OFFSET_DELTA_Z
    }
end


--------------------------------------------------------------------------------
-- User Interface
--------------------------------------------------------------------------------

UI_SHELF_ID = 1001
UI_SLOT_ID  = 1002

TOTAL_SLOTS   = 32
TOTAL_SHELVES = 8


UI_XML = [[
<ui
    closeable="false"
>
    <group layout="form">
        <label
            text="Shelf"
        />
        <spinbox
            id="]] .. UI_SHELF_ID .. [["
            minimum="1"
            maximum="]] .. TOTAL_SHELVES .. [["
        />

        <label
            text="Cartridge"
        />
	<spinbox
            id="]] .. UI_SLOT_ID .. [["
            minimum="1"
            maximum="]] .. TOTAL_SLOTS .. [["
        />

        <label />
        <button
            text="Load"
            on-click="on_load_request"
	/>

        <label />
        <button
            text="Eject"
            on-click="on_eject_request"
	/>
    </group>
</ui>
]]


--------------------------------------------------------------------------------
-- User Interface Callbacks
--------------------------------------------------------------------------------

function on_load_request(ui, id)
    if g_state.state ~= STATE_HALT_UNLOADED then
        log_warning("Can't load cartridge now.")
        return
    end

    shelf = simUI.getSpinboxValue(ui, UI_SHELF_ID)
    slot  = simUI.getSpinboxValue(ui, UI_SLOT_ID)

    g_state = {
        state = STATE_PICKING_PREPARING,
        target = slot_position(shelf - 1, slot - 1)
    }
end


function on_eject_request(ui, id)
    if g_state.state ~= STATE_HALT_LOADED then
        log_warning("Can't eject cartridge now.")
        return
    end

    g_state.state = STATE_UNLOADING_PREPARING
end


--------------------------------------------------------------------------------
-- Object Parenting Helpers
--------------------------------------------------------------------------------

function attach_objects(parent, child)
    sim.setObjectParent(child, parent, true)
end


function detach_object(obj)
    sim.setObjectParent(obj, -1, true)
end


--------------------------------------------------------------------------------
-- Proximity Sensor Helpers
--------------------------------------------------------------------------------

function get_proximity_sensor_target(sensor)
    local result, _, _, target, _ = sim.readProximitySensor(sensor)
    if result == 1 then
        return target
    end
    return nil
end


--------------------------------------------------------------------------------
-- Joints Helpers
--------------------------------------------------------------------------------

JOINT_PROXIMITY_TOLERANCE = 0.01

function is_target_reached(position, target)
    return math.abs(position - target) <= JOINT_PROXIMITY_TOLERANCE
end


function move_joint(joint, target)
    sim.setJointTargetPosition(joint, target)
end


function get_joint_position(joint)
    return sim.getJointPosition(joint)
end


function has_joint_reached_target(joint, target)
    local joint_position = get_joint_position(joint)
    return is_target_reached(joint_position, target)
end


--------------------------------------------------------------------------------
-- Logging Helpers
--------------------------------------------------------------------------------

function log_error(msg)
    sim.addLog(sim.verbosity_scripterrors, msg)
end


function log_warning(msg)
    sim.addLog(sim.verbosity_scriptwarnings, msg)
end


function log_info(msg)
    sim.addLog(sim.verbosity_scriptinfos, msg)
end


function fatal(msg)
    log_error("Stopping simulation due to the following error:")
    log_error(msg)
    sim.stopSimulation()
end
