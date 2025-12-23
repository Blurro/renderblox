// LaunchRenderBlox.cpp : Defines the entry point for the application.
//

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>
#include <cstdlib>
#include <cstdio>

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;                        // current instance
WCHAR szTitle[MAX_LOADSTRING];          // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];    // the main window class name

static void ShowError(const wchar_t *msg)
{
  MessageBoxW(NULL, msg, L"LaunchRenderBlox", MB_ICONERROR | MB_OK);
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance,
                      _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
  UNREFERENCED_PARAMETER(hPrevInstance);
  UNREFERENCED_PARAMETER(lpCmdLine);
  UNREFERENCED_PARAMETER(nCmdShow);

  hInst = hInstance;

  const wchar_t *dirPath = L".\\rbtools";
  const wchar_t *exePath = L".\\rbtools\\qrenderdoc.exe";

  DWORD attrs = GetFileAttributesW(dirPath);
  if(attrs == INVALID_FILE_ATTRIBUTES || !(attrs & FILE_ATTRIBUTE_DIRECTORY))
  {
    ShowError(L"rbtools directory not found");
    return 1;
  }

  if(GetFileAttributesW(exePath) == INVALID_FILE_ATTRIBUTES)
  {
    ShowError(L"qrenderdoc.exe not found in rbtools");
    return 1;
  }

  SHELLEXECUTEINFOW sei = {};
  sei.cbSize = sizeof(sei);
  sei.lpFile = exePath;
  sei.nShow = SW_SHOWNORMAL;

  if(!ShellExecuteExW(&sei))
  {
    ShowError(L"failed to launch qrenderdoc.exe");
    return 1;
  }

  return 0;
}