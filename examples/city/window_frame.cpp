#include "window_frame.h"

#include "imgui/imgui.h"
#include <dear-imgui/imgui_internal.h>

#include <atomic>

#if FRONTIER_CITY_SDL_WINDOW_FRAME
#include <SDL2/SDL.h>
#endif

namespace city
{
namespace
{
constexpr int kTitleHeight = 32;
constexpr int kBorder = 6;
constexpr int kButtonWidth = 36;
enum class Action { None, Minimize, MaximizeRestore, Close };
std::atomic<bool> enabled{false};
std::atomic<bool> fullscreen{false};
std::atomic<bool> maximized{false};
std::atomic<Action> requestedAction{Action::None};

#if FRONTIER_CITY_SDL_WINDOW_FRAME
SDL_HitTestResult SDLCALL hitTest(SDL_Window* window, const SDL_Point* point, void*)
{
    const Uint32 flags = SDL_GetWindowFlags(window);
    if ((flags & SDL_WINDOW_FULLSCREEN) != 0)
        return SDL_HITTEST_NORMAL;
    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    if ((flags & SDL_WINDOW_RESIZABLE) != 0 &&
        (flags & SDL_WINDOW_MAXIMIZED) == 0)
    {
        const bool left = point->x < kBorder;
        const bool right = point->x >= width - kBorder;
        const bool top = point->y < kBorder;
        const bool bottom = point->y >= height - kBorder;
        if (top && left) return SDL_HITTEST_RESIZE_TOPLEFT;
        if (top && right) return SDL_HITTEST_RESIZE_TOPRIGHT;
        if (bottom && left) return SDL_HITTEST_RESIZE_BOTTOMLEFT;
        if (bottom && right) return SDL_HITTEST_RESIZE_BOTTOMRIGHT;
        if (top) return SDL_HITTEST_RESIZE_TOP;
        if (bottom) return SDL_HITTEST_RESIZE_BOTTOM;
        if (left) return SDL_HITTEST_RESIZE_LEFT;
        if (right) return SDL_HITTEST_RESIZE_RIGHT;
    }
    // Leave all three buttons to ImGui. Only the caption initiates a move.
    if (point->y < kTitleHeight && point->x < width - kBorder - 3 * kButtonWidth)
        return SDL_HITTEST_DRAGGABLE;
    const SDL_Keymod modifiers = SDL_GetModState();
    if ((modifiers & KMOD_ALT) != 0)
    {
        if ((modifiers & KMOD_SHIFT) != 0 && (flags & SDL_WINDOW_RESIZABLE) != 0 &&
            (flags & SDL_WINDOW_MAXIMIZED) == 0)
        {
            const bool left = point->x < width / 2;
            return point->y < height / 2
                ? (left ? SDL_HITTEST_RESIZE_TOPLEFT : SDL_HITTEST_RESIZE_TOPRIGHT)
                : (left ? SDL_HITTEST_RESIZE_BOTTOMLEFT : SDL_HITTEST_RESIZE_BOTTOMRIGHT);
        }
        return SDL_HITTEST_DRAGGABLE;
    }
    return SDL_HITTEST_NORMAL;
}
#endif
}

void enableWindowFrame(bool value)
{
    enabled.store(value);
}

float windowFrameHeight()
{
    return enabled.load() && !fullscreen.load() ? float(kTitleHeight) : 0.0f;
}

void drawWindowFrame()
{
    if (windowFrameHeight() == 0.0f)
        return;
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_NoCollapse;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.13f, 0.17f, 0.21f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    if (ImGui::BeginViewportSideBar("##CityWindowTitle", viewport, ImGuiDir_Up,
                                     float(kTitleHeight), flags))
    {
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const ImVec2 origin = ImGui::GetWindowPos();
        const float buttonStart = ImGui::GetWindowWidth() - kBorder - 3 * kButtonWidth;
        draw->PushClipRect(origin, ImVec2(origin.x + buttonStart, origin.y + kTitleHeight), true);
        draw->AddText(ImVec2(origin.x + 12.0f, origin.y + (kTitleHeight - ImGui::GetFontSize()) * 0.5f),
                      IM_COL32(235, 239, 244, 255), "Frontier - Dynamic City");
        draw->PopClipRect();
        const Action actions[] = {Action::Minimize, Action::MaximizeRestore, Action::Close};
        const char* labels[] = {"Minimize", maximized.load() ? "Restore" : "Maximize", "Close"};
        for (int index = 0; index < 3; ++index)
        {
            const float x = buttonStart + float(index * kButtonWidth);
            ImGui::SetCursorPos(ImVec2(x, float(kBorder)));
            if (ImGui::InvisibleButton(labels[index], ImVec2(float(kButtonWidth), float(kTitleHeight - kBorder))))
                requestedAction.store(actions[index]);
            if (ImGui::IsItemHovered())
            {
                draw->AddRectFilled(ImVec2(origin.x + x, origin.y + kBorder),
                                    ImVec2(origin.x + x + kButtonWidth, origin.y + kTitleHeight),
                                    index == 2 ? IM_COL32(190, 50, 55, 255) : IM_COL32(70, 85, 100, 255));
                ImGui::SetTooltip("%s", labels[index]);
            }
            const float cx = origin.x + x + kButtonWidth * 0.5f;
            const float cy = origin.y + (kTitleHeight + kBorder) * 0.5f;
            constexpr ImU32 color = IM_COL32(235, 239, 244, 255);
            if (index == 0)
                draw->AddLine(ImVec2(cx - 5, cy + 3), ImVec2(cx + 5, cy + 3), color);
            else if (index == 1)
            {
                if (maximized.load())
                    draw->AddRect(ImVec2(cx - 2, cy - 6), ImVec2(cx + 6, cy + 2), color);
                draw->AddRect(ImVec2(cx - 5, cy - 3), ImVec2(cx + 3, cy + 5), color);
            }
            else
            {
                draw->AddLine(ImVec2(cx - 4, cy - 4), ImVec2(cx + 4, cy + 4), color);
                draw->AddLine(ImVec2(cx - 4, cy + 4), ImVec2(cx + 4, cy - 4), color);
            }
        }
        ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());
    }
    ImGui::End();
    const ImGuiDir sides[] = {ImGuiDir_Left, ImGuiDir_Right, ImGuiDir_Down};
    const char* names[] = {"##CityWindowLeft", "##CityWindowRight", "##CityWindowBottom"};
    for (int index = 0; index < 3; ++index)
    {
        ImGui::BeginViewportSideBar(names[index], viewport, sides[index], float(kBorder),
                                    flags | ImGuiWindowFlags_NoInputs);
        ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());
        ImGui::End();
    }
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
}

void serviceWindowFrame(void* sdlWindow)
{
#if FRONTIER_CITY_SDL_WINDOW_FRAME
    if (!enabled.load())
        return;
    auto* window = static_cast<SDL_Window*>(sdlWindow);
    static SDL_Window* configuredWindow = nullptr;
    if (configuredWindow != window)
    {
        if (SDL_SetWindowHitTest(window, hitTest, nullptr) != 0)
        {
            SDL_Log("Frontier: window frame hit test failed: %s", SDL_GetError());
            enabled.store(false);
            return;
        }
        SDL_SetWindowMinimumSize(window, 360, 240);
        SDL_SetWindowBordered(window, SDL_FALSE);
        configuredWindow = window;
        SDL_Log("Frontier: application window frame active (drag title to move, edges to resize).");
    }
    const Uint32 flags = SDL_GetWindowFlags(window);
    fullscreen.store((flags & SDL_WINDOW_FULLSCREEN) != 0);
    maximized.store((flags & SDL_WINDOW_MAXIMIZED) != 0);
    switch (requestedAction.exchange(Action::None))
    {
    case Action::Minimize: SDL_MinimizeWindow(window); break;
    case Action::MaximizeRestore:
        if ((flags & SDL_WINDOW_MAXIMIZED) != 0) SDL_RestoreWindow(window);
        else SDL_MaximizeWindow(window);
        break;
    case Action::Close:
    {
        SDL_Event event{};
        event.type = SDL_QUIT;
        SDL_PushEvent(&event);
        break;
    }
    case Action::None: break;
    }
#else
    (void)sdlWindow;
#endif
}
}
