// Copyright 2008 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

class PointerWrap;

void VideoCommon_DoState(PointerWrap& p);

// FFNY rollback resync: while set, a state save leaves out the EFB contents and the texture cache's EFB copies
// (as with "Save Texture Cache to State" off). They depend on the video backend / resolution and are large; the
// receiving Dolphin clears its EFB and decodes textures from RAM again instead.
extern bool g_state_skip_texture_cache;
