-- -*- mode: Lua; lua-indent-level: 4; -*-

OBJECT_CARTRIDGE = sim.getObject('/Cartridge')
OBJECT_RACK      = sim.getObject('/Rack')

SHELVES = 8
SLOTS = 32

CARTRIDGE_SIZE_X = 0.02155
CARTRIDGE_DISTANCE = 0.05
CARTRIDGE_CENTER_X = CARTRIDGE_SIZE_X / 2

SHELF_BORDER_X = 0.02

OFFSET_BASE_X = SHELF_BORDER_X + CARTRIDGE_DISTANCE + CARTRIDGE_CENTER_X
OFFSET_DELTA_X = CARTRIDGE_DISTANCE + CARTRIDGE_SIZE_X

OFFSET_Y = -0.050

OFFSET_BASE_Z = 0.20
OFFSET_DELTA_Z = 0.15


function cartridge_position(shelf, slot)
    -- first shelf = 0, last = 31
    -- first slot  = 0, last =  7
    return {
	OFFSET_BASE_X + slot * OFFSET_DELTA_X,
	OFFSET_Y,
	OFFSET_BASE_Z + shelf * OFFSET_DELTA_Z
    }
end


for shelf = 0, SHELVES - 1 do
    for slot = 0, SLOTS - 1 do
	local copies = sim.copyPasteObjects({ OBJECT_CARTRIDGE }, 32)
	sim.setObjectPosition(
	    copies[1],
	    sim.handle_world,
	    cartridge_position(shelf, slot)
	)
	sim.setObjectParent(copies[1], OBJECT_RACK, true)
    end
end
