#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <windowsx.h>
#include <cmath>
#include <string>
#include <vector>
#include <cstdlib>

struct Vec2 {
    float x, y;
    Vec2(float _x = 0.0f, float _y = 0.0f) : x(_x), y(_y) {}
    Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
    float length() const { return std::sqrt(x * x + y * y); }
    Vec2 normalized() const {
        float l = length();
        return l > 0.0001f ? Vec2(x / l, y / l) : Vec2(0, 0);
    }
};

// Anatomical Base Anchor & Segment Dimensions
const Vec2 THORAX_BASE(170.0f, 430.0f);
const float L_COXA = 85.0f;
const float L_FEMUR = 145.0f;
const float L_TIBIA = 125.0f;

// Simulation State
Vec2 flyPos(600.0f, 260.0f);
Vec2 flyVel(0.0f, 0.0f);
Vec2 strikeTarget(600.0f, 260.0f);
bool isStriking = false;
float strikeProgress = 0.0f;
int catches = 0;
int escapes = 0;
float strikeSpeedCurrent = 0.0f;

// 2-Link Analytic Inverse Kinematics
void solveArmIK(Vec2 base, Vec2 target, float l1, float l2, Vec2& elbow, Vec2& endEffector) {
    Vec2 d = target - base;
    float dist = d.length();
    float maxReach = l1 + l2 - 2.0f;
    if (dist > maxReach) dist = maxReach;
    if (dist < 20.0f) dist = 20.0f;

    float angleBase = std::atan2(d.y, d.x);
    float cosElbow = (dist * dist - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    if (cosElbow < -1.0f) cosElbow = -1.0f;
    if (cosElbow > 1.0f) cosElbow = 1.0f;
    float angleElbow = -std::acos(cosElbow);

    float angleShoulder = angleBase - std::atan2(l2 * std::sin(angleElbow), l1 + l2 * std::cos(angleElbow));

    elbow = base + Vec2(l1 * std::cos(angleShoulder), l1 * std::sin(angleShoulder));
    endEffector = elbow + Vec2(l2 * std::cos(angleShoulder + angleElbow), l2 * std::sin(angleShoulder + angleElbow));
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_LBUTTONDOWN:
            if (!isStriking) {
                isStriking = true;
                strikeTarget = flyPos;
                strikeProgress = 0.0f;
            }
            break;
        case WM_RBUTTONDOWN:
            // Right-click re-spawns fly at cursor
            flyPos.x = (float)LOWORD(lParam);
            flyPos.y = (float)HIWORD(lParam);
            flyVel = Vec2(0, 0);
            break;
        case WM_DESTROY:
            PostQuitMessage(0);
            break;
        default:
            return DefWindowProc(hwnd, msg, wParam, lParam);
    }
    return 0;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    WNDCLASS wc = {0};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = "ApexMantisClass";
    wc.hCursor = LoadCursor(NULL, IDC_CROSS);
    RegisterClass(&wc);

    HWND hwnd = CreateWindow("ApexMantisClass", "APEX STRIKE // Bio-Kinetic Mantis Simulation",
                             WS_OVERLAPPEDWINDOW | WS_VISIBLE, 80, 80, 1024, 700, NULL, NULL, hInstance, NULL);

    MSG msg;
    DWORD lastTick = GetTickCount();

    while (true) {
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) return 0;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        DWORD currentTick = GetTickCount();
        float dt = (currentTick - lastTick) / 1000.0f;
        if (dt > 0.05f) dt = 0.05f;
        lastTick = currentTick;

        // --- Autonomous Fly AI (Erratic Brownian flight + Looming Evasion) ---
        flyVel.x += ((rand() % 100 - 50) / 50.0f) * 45.0f * dt;
        flyVel.y += ((rand() % 100 - 50) / 50.0f) * 45.0f * dt;
        flyVel = flyVel * 0.96f; // Air damping

        // Restrict fly flight arena
        if (flyPos.x < 360.0f) flyVel.x += 120.0f * dt;
        if (flyPos.x > 940.0f) flyVel.x -= 120.0f * dt;
        if (flyPos.y < 120.0f) flyVel.y += 120.0f * dt;
        if (flyPos.y > 580.0f) flyVel.y -= 120.0f * dt;

        // --- Arm Kinematics & Strike Solver ---
        Vec2 coxaEnd = THORAX_BASE + Vec2(L_COXA * 0.65f, -L_COXA * 0.75f);
        Vec2 restPos = coxaEnd + Vec2(50.0f, -40.0f);
        Vec2 clawGoal = restPos;

        if (isStriking) {
            strikeProgress += dt * 10.5f; // Fast spring release (~95 ms)
            if (strikeProgress < 1.0f) {
                // Strike extension phase
                clawGoal = restPos + (strikeTarget - restPos) * strikeProgress;
                strikeSpeedCurrent = (strikeTarget - restPos).length() * 10.5f;

                // Fly escape reflex triggered by looming danger
                Vec2 threatDist = clawGoal - flyPos;
                if (threatDist.length() < 95.0f) {
                    flyVel = flyVel + (flyPos - clawGoal).normalized() * 380.0f * dt;
                }
            } else if (strikeProgress < 2.0f) {
                // Retract phase
                clawGoal = strikeTarget - (strikeTarget - restPos) * (strikeProgress - 1.0f);
            } else {
                isStriking = false;
                strikeProgress = 0.0f;
                strikeSpeedCurrent = 0.0f;
            }

            // Capture verification at peak extension
            if (strikeProgress >= 0.95f && strikeProgress <= 1.15f) {
                if ((strikeTarget - flyPos).length() < 34.0f) {
                    catches++;
                    // Reset fly
                    flyPos = Vec2(650.0f + (rand() % 200), 200.0f + (rand() % 200));
                    flyVel = Vec2(0, 0);
                    isStriking = false;
                } else if (strikeProgress > 1.1f && (strikeTarget - flyPos).length() >= 34.0f) {
                    escapes++;
                }
            }
        }
        flyPos = flyPos + flyVel;

        Vec2 femurEnd, clawTip;
        solveArmIK(coxaEnd, clawGoal, L_FEMUR, L_TIBIA, femurEnd, clawTip);

        // --- Rendering Setup ---
        RECT rc;
        GetClientRect(hwnd, &rc);
        HDC hdc = GetDC(hwnd);
        HDC memDC = CreateCompatibleDC(hdc);
        HBITMAP memBitmap = CreateCompatibleBitmap(hdc, rc.right, rc.bottom);
        SelectObject(memDC, memBitmap);

        // Bio-Lab Grid Background
        HBRUSH bgBrush = CreateSolidBrush(RGB(14, 18, 22));
        FillRect(memDC, &rc, bgBrush);
        DeleteObject(bgBrush);

        HPEN gridPen = CreatePen(PS_SOLID, 1, RGB(22, 28, 34));
        SelectObject(memDC, gridPen);
        for (int x = 0; x < rc.right; x += 50) {
            MoveToEx(memDC, x, 0, NULL); LineTo(memDC, x, rc.bottom);
        }
        for (int y = 0; y < rc.bottom; y += 50) {
            MoveToEx(memDC, 0, y, NULL); LineTo(memDC, rc.right, y);
        }
        DeleteObject(gridPen);

        // --- Head & Vision Tracking ---
        Vec2 neckBase = THORAX_BASE + Vec2(25.0f, -65.0f);
        Vec2 headDir = (flyPos - neckBase).normalized();
        Vec2 headTip = neckBase + headDir * 35.0f;
        Vec2 headPerp(-headDir.y, headDir.x);

        // Triangular Head Vertices
        POINT headPts[3] = {
            { (int)(headTip.x), (int)(headTip.y) },
            { (int)(neckBase.x + headPerp.x * 22.0f), (int)(neckBase.y + headPerp.y * 22.0f) },
            { (int)(neckBase.x - headPerp.x * 22.0f), (int)(neckBase.y - headPerp.y * 22.0f) }
        };

        HBRUSH headBrush = CreateSolidBrush(RGB(55, 160, 65));
        SelectObject(memDC, headBrush);
        Polygon(memDC, headPts, 3);
        DeleteObject(headBrush);

        // Stereoscopic Compound Eyes
        HBRUSH eyeBrush = CreateSolidBrush(RGB(140, 230, 90));
        SelectObject(memDC, eyeBrush);
        Vec2 eyeLeft = neckBase + headPerp * 18.0f;
        Vec2 eyeRight = neckBase - headPerp * 18.0f;
        Ellipse(memDC, (int)eyeLeft.x - 7, (int)eyeLeft.y - 7, (int)eyeLeft.x + 7, (int)eyeLeft.y + 7);
        Ellipse(memDC, (int)eyeRight.x - 7, (int)eyeRight.y - 7, (int)eyeRight.x + 7, (int)eyeRight.y + 7);
        DeleteObject(eyeBrush);

        // Pseudopupils (Focusing on prey)
        HBRUSH pupilBrush = CreateSolidBrush(RGB(15, 30, 15));
        SelectObject(memDC, pupilBrush);
        Vec2 pLeft = eyeLeft + headDir * 3.0f;
        Vec2 pRight = eyeRight + headDir * 3.0f;
        Ellipse(memDC, (int)pLeft.x - 3, (int)pLeft.y - 3, (int)pLeft.x + 3, (int)pLeft.y + 3);
        Ellipse(memDC, (int)pRight.x - 3, (int)pRight.y - 3, (int)pRight.x + 3, (int)pRight.y + 3);
        DeleteObject(pupilBrush);

        // Dynamic 3D Vision Laser Cones
        HPEN conePen = CreatePen(PS_DOT, 1, isStriking ? RGB(220, 60, 60) : RGB(45, 120, 80));
        SelectObject(memDC, conePen);
        MoveToEx(memDC, (int)eyeLeft.x, (int)eyeLeft.y, NULL);
        LineTo(memDC, (int)flyPos.x, (int)flyPos.y);
        MoveToEx(memDC, (int)eyeRight.x, (int)eyeRight.y, NULL);
        LineTo(memDC, (int)flyPos.x, (int)flyPos.y);
        DeleteObject(conePen);

        // Mantis Thorax (Chitin Plates)
        HPEN thoraxPen = CreatePen(PS_SOLID, 12, RGB(45, 135, 55));
        SelectObject(memDC, thoraxPen);
        MoveToEx(memDC, (int)THORAX_BASE.x - 65, (int)THORAX_BASE.y + 115, NULL);
        LineTo(memDC, (int)THORAX_BASE.x, (int)THORAX_BASE.y);
        LineTo(memDC, (int)neckBase.x, (int)neckBase.y);
        DeleteObject(thoraxPen);

        // --- Raptorial Leg with Barbed Spines ---
        // Coxa
        HPEN coxaPen = CreatePen(PS_SOLID, 11, RGB(70, 185, 80));
        SelectObject(memDC, coxaPen);
        MoveToEx(memDC, (int)THORAX_BASE.x, (int)THORAX_BASE.y, NULL);
        LineTo(memDC, (int)coxaEnd.x, (int)coxaEnd.y);
        DeleteObject(coxaPen);

        // Femur
        HPEN femurPen = CreatePen(PS_SOLID, 10, RGB(95, 215, 90));
        SelectObject(memDC, femurPen);
        MoveToEx(memDC, (int)coxaEnd.x, (int)coxaEnd.y, NULL);
        LineTo(memDC, (int)femurEnd.x, (int)femurEnd.y);
        DeleteObject(femurPen);

        // Tibia (Scythe)
        HPEN tibiaPen = CreatePen(PS_SOLID, 6, RGB(180, 255, 110));
        SelectObject(memDC, tibiaPen);
        MoveToEx(memDC, (int)femurEnd.x, (int)femurEnd.y, NULL);
        LineTo(memDC, (int)clawTip.x, (int)clawTip.y);
        DeleteObject(tibiaPen);

        // Femur Spines
        HPEN spinePen = CreatePen(PS_SOLID, 2, RGB(220, 250, 160));
        SelectObject(memDC, spinePen);
        Vec2 femurDir = (femurEnd - coxaEnd).normalized();
        Vec2 femurNormal(-femurDir.y, femurDir.x);
        for (int s = 1; s <= 4; ++s) {
            Vec2 spBase = coxaEnd + femurDir * (s * 26.0f);
            Vec2 spTip = spBase + femurNormal * 12.0f;
            MoveToEx(memDC, (int)spBase.x, (int)spBase.y, NULL);
            LineTo(memDC, (int)spTip.x, (int)spTip.y);
        }
        DeleteObject(spinePen);

        // Target Lock Reticle
        HPEN reticlePen = CreatePen(PS_SOLID, 1, RGB(230, 80, 80));
        SelectObject(memDC, reticlePen);
        MoveToEx(memDC, (int)flyPos.x - 14, (int)flyPos.y, NULL); LineTo(memDC, (int)flyPos.x + 14, (int)flyPos.y);
        MoveToEx(memDC, (int)flyPos.x, (int)flyPos.y - 14, NULL); LineTo(memDC, (int)flyPos.x, (int)flyPos.y + 14);
        DeleteObject(reticlePen);

        // Fly Body & Wings
        HBRUSH flyBody = CreateSolidBrush(RGB(240, 50, 50));
        SelectObject(memDC, flyBody);
        Ellipse(memDC, (int)flyPos.x - 6, (int)flyPos.y - 6, (int)flyPos.x + 6, (int)flyPos.y + 6);
        DeleteObject(flyBody);

        // Telemetry HUD
        SetTextColor(memDC, RGB(180, 240, 190));
        SetBkMode(memDC, TRANSPARENT);
        float distanceToFly = (coxaEnd - flyPos).length();
        
        std::string s1 = "// APEX BIOMECHANICAL RIG // STATUS: " + std::string(isStriking ? "STRIKING (SPRING ACTIVATION)" : "CALCULATING DEPTH MAP");
        std::string s2 = "TARGET DISTANCE: " + std::to_string((int)distanceToFly) + " mm | MAX REACH: 270 mm";
        std::string s3 = "STRIKE ACCELERATION: " + std::to_string((int)strikeSpeedCurrent) + " mm/s | CAPTURES: " + std::to_string(catches);
        std::string s4 = "[LEFT-CLICK: EXECUTE STRIKE]   [RIGHT-CLICK: RE-POSITION PREY]";

        TextOut(memDC, 25, 20, s1.c_str(), (int)s1.length());
        TextOut(memDC, 25, 42, s2.c_str(), (int)s2.length());
        TextOut(memDC, 25, 64, s3.c_str(), (int)s3.length());
        SetTextColor(memDC, RGB(120, 170, 130));
        TextOut(memDC, 25, 90, s4.c_str(), (int)s4.length());

        BitBlt(hdc, 0, 0, rc.right, rc.bottom, memDC, 0, 0, SRCCOPY);

        DeleteObject(memBitmap);
        DeleteDC(memDC);
        ReleaseDC(hwnd, hdc);

        Sleep(16); // ~60 FPS
    }
    return 0;
}