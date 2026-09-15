#pragma once

namespace city
{
// SDL window operations run on the entry/event thread. ImGui runs on the
// application thread; the implementation exchanges only atomic state/actions.
void enableWindowFrame(bool enabled);
float windowFrameHeight();
void drawWindowFrame();
void serviceWindowFrame(void* sdlWindow);
}
