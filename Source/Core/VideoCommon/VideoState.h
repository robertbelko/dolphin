// Copyright 2008 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

class PointerWrap;

void VideoCommon_DoState(PointerWrap& p);

// FFNY rollback resync: while set, a state save leaves out the EFB contents and the texture cache's EFB copies
// (as with "Save Texture Cache to State" off). They depend on the video backend / resolution and are large; the
// receiving Dolphin clears its EFB and decodes textures from RAM again instead.
extern bool g_state_skip_texture_cache;

// FFNY rollback resync: while set, the state being loaded comes from another player's Dolphin. Parts that depend on
// that player's own settings rather than on the game (the free look camera type) are read and dropped instead of
// aborting the load (24 October: a different free look camera type - even with free look off - made every resync
// fail at "PixelShaderManager", i.e. in VertexShaderManager's free look part).
extern bool g_state_foreign_load;
