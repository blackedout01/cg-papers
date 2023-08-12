#include <stdio.h>
#include <math.h>
#include "Eigen/Dense"
#include "SDL.h"
#include "glad/glad.h"

#include <iostream>

#define ArrayCount(X) (sizeof(X)/sizeof(*X))

#ifndef GL_STACK_OVERFLOW
#define GL_STACK_OVERFLOW 0x0503
#endif
#ifndef GL_STACK_UNDERFLOW
#define GL_STACK_UNDERFLOW 0x0504
#endif
#ifndef GL_OUT_OF_MEMORY
#define GL_OUT_OF_MEMORY 0x0505
#endif
#ifndef GL_INVALID_FRAMEBUFFER_OPERATION
#define GL_INVALID_FRAMEBUFFER_OPERATION 0x0506
#endif
#ifndef GL_CONTEXT_LOST
#define GL_CONTEXT_LOST 0x0507
#endif
#ifndef GL_TABLE_TOO_LARGE1
#define GL_TABLE_TOO_LARGE1 0x8031
#endif

static const char *ErrorStringOpenGL(GLenum Error) {
    switch(Error) {
    case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
    case GL_STACK_OVERFLOW: return "GL_STACK_OVERFLOW";
    case GL_STACK_UNDERFLOW: return "GL_STACK_UNDERFLOW";
    case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
    case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
    case GL_CONTEXT_LOST: return "GL_CONTEXT_LOST";
    case GL_TABLE_TOO_LARGE1: return "GL_TABLE_TOO_LARGE1";
    default: return "unknwon";
    }
}

static const char *ErrorDescriptionOpenGL(GLenum Error) {
    // NOTE(blackedout): Taken from https://www.khronos.org/opengl/wiki/OpenGL_Error 2023-04-04
    switch(Error) {
    case GL_INVALID_ENUM: return "Given when an enumeration parameter is not a legal enumeration for that function. This is given only for local problems; if the spec allows the enumeration in certain circumstances, where other parameters or state dictate those circumstances, then GL_INVALID_OPERATION is the result instead.";
    case GL_INVALID_VALUE: return "Given when a value parameter is not a legal value for that function. This is only given for local problems; if the spec allows the value in certain circumstances, where other parameters or state dictate those circumstances, then GL_INVALID_OPERATION is the result instead.";
    case GL_INVALID_OPERATION: return "Given when the set of state for a command is not legal for the parameters given to that command. It is also given for commands where combinations of parameters define what the legal parameters are.";
    case GL_STACK_OVERFLOW: return "Given when a stack pushing operation cannot be done because it would overflow the limit of that stack's size.";
    case GL_STACK_UNDERFLOW: return "Given when a stack popping operation cannot be done because the stack is already at its lowest point.";
    case GL_OUT_OF_MEMORY: return "Given when performing an operation that can allocate memory, and the memory cannot be allocated. The results of OpenGL functions that return this error are undefined; it is allowable for partial execution of an operation to happen in this circumstance.";
    case GL_INVALID_FRAMEBUFFER_OPERATION: return "Given when doing anything that would attempt to read from or write/render to a framebuffer that is not complete.";
    case GL_CONTEXT_LOST: return "Given if the OpenGL context has been lost, due to a graphics card reset (with OpenGL 4.5 or ARB_KHR_robustness).";
    case GL_TABLE_TOO_LARGE1: return "Part of the ARB_imaging extension1. 1 These error codes are deprecated in 3.0 and removed in 3.1 core and above.";
    default: return "";
    }
}

static void CheckOpenGL(const char *CallStr) {
    GLenum Error = glGetError();
    if(Error != GL_NO_ERROR) {
        fprintf(stderr, "%s\n", CallStr);
        
        int I = 0;
        do {
            fprintf(stderr, "\t[%d]: %s (%d) %s\n", I++, ErrorStringOpenGL(Error), Error, ErrorDescriptionOpenGL(Error));
        } while((Error = glGetError()) != GL_NO_ERROR);
    }
}

// NOTE(blackedout): Do NOT call this function as part of a single statement if, while, etc.
#define glCheck(X) X; do { CheckOpenGL(#X); } while(0)

typedef Eigen::Vector2f v2;
typedef Eigen::Vector3f v3;
typedef Eigen::Matrix2f m2;
typedef Eigen::Matrix3f m3;

static m2 Rotation(float Angle) {
    v2 X(cosf(Angle), sinf(Angle));
    v2 Y(-sinf(Angle), cosf(Angle));
    
    m2 R;
    R << X, Y;
    return R;
}

static m3 RotationX(float Angle) {
    float C = cosf(Angle);
    float S = sinf(Angle);
    v3 X(1.0f, 0.0f, 0.0f);
    v3 Y(0.0f, C, S);
    v3 Z(0.0f, -S, C);
    
    m3 R;
    R << X, Y, Z;
    return R;
}

static m3 RotationY(float Angle) {
    float C = cosf(Angle);
    float S = sinf(Angle);
    v3 X(C, 0.0f, -S);
    v3 Y(0.0f, 1.0f, 0.0f);
    v3 Z(S, 0.0f, C);
    
    m3 R;
    R << X, Y, Z;
    return R;
}

static m3 RotationZ(float Angle) {
    float C = cosf(Angle);
    float S = sinf(Angle);
    v3 X(C, S, 0.0f);
    v3 Y(-S, C, 0.0f);
    v3 Z(0.0f, 0.0f, 1.0f);
    
    m3 R;
    R << X, Y, Z;
    return R;
}

struct vertex {
    v3 Position;
    v3 Color;
};

static char *ReadFile(const char *FileName) {
    FILE *File = fopen(FileName, "r");
    fseek(File, 0, SEEK_END);
    long int FileSize = ftell(File);
    fseek(File, 0, SEEK_SET);
    
    char *Bytes = (char *)malloc(FileSize + 1);
    Bytes[FileSize] = 0;
    fread(Bytes, FileSize, 1, File);
    return Bytes;
}

static vertex MeshData[] = {
    { { 0.5f,  0.5f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
    { { -0.5f,  0.5f, 0.0f }, { 1.0f, 1.0f, 0.0f } },
    { { -0.5f, -0.5f, 0.0f }, { 0.0f, 1.0f, 0.0f } },
    
    { { 0.5f,  0.5f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
    { { 0.5f, -0.5f, 0.0f }, { 0.0f, 0.0f, 1.0f } },
    { { -0.5f, -0.5f, 0.0f }, { 0.0f, 1.0f, 0.0f } },
};

static int CompileShader(GLuint *Shader, const char *VertexShader, const char *FragmentShader) {
    int Success0;

    GLuint VID = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(VID, 1, (const GLchar * const*)&VertexShader, 0);
    glCompileShader(VID);
    
    glGetShaderiv(VID, GL_COMPILE_STATUS, &Success0);
    if (Success0 == GL_FALSE) {
        GLchar Buf[1024] = {0};
        GLsizei Length = 0;
        glGetShaderInfoLog(VID, ArrayCount(Buf), &Length, Buf);
        fprintf(stderr, "%.*s", Length, Buf);
        return 1;
    }

    GLuint FID = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(FID, 1, (const GLchar * const*)&FragmentShader, 0);
    glCompileShader(FID);
    
    glGetShaderiv(FID, GL_COMPILE_STATUS, &Success0);
    if (Success0 == GL_FALSE) {
        GLchar Buf[1024] = {0};
        GLsizei Length = 0;
        glGetShaderInfoLog(FID, ArrayCount(Buf), &Length, Buf);
        fprintf(stderr, "%.*s", Length, Buf);
        return 1;
    }

    GLuint Program = glCreateProgram();
    glAttachShader(Program, VID);
    glAttachShader(Program, FID);
    glLinkProgram(Program);

    glGetProgramiv(Program, GL_LINK_STATUS, &Success0);
    if (Success0 == GL_FALSE) {
        GLchar Buf[1024] = {0};
        GLsizei Length = 0;
        glGetProgramInfoLog(Program, ArrayCount(Buf), &Length, Buf);
        fprintf(stderr, "%.*s", Length, Buf);
        return 1;
    }

    glDetachShader(Program, VID);
    glDetachShader(Program, FID);
    glDeleteShader(VID);
    glDeleteShader(FID);

    *Shader = Program;
    return 0;
}

static int LoadShader(GLuint *Shader, const char *VertexPath, const char *FragmentPath) {
    char *VertexCode = ReadFile(VertexPath);
    char *FragmentCode = ReadFile(FragmentPath);
    
    int Error = CompileShader(Shader, VertexCode, FragmentCode);
    
    free(VertexCode);
    free(FragmentCode);
    
    return Error;
}

typedef struct {
    Sint16 L;
    Sint16 R;
} audio_sample;

typedef struct {
    int Freq;
    long SamplesDone;
} audio_data;

static void MyAudioCallback(void *Userdata, Uint8 *Stream, int Len) {
    audio_data *Audio = (audio_data *)Userdata;
    int SampleCount = Len/sizeof(audio_sample);
    
    audio_sample *Samples = (audio_sample *)Stream;
    for(int I = 0; I < SampleCount; ++I) {
        
        double Time = (Audio->SamplesDone + I)/(double)(Audio->Freq);
        
        double ValueF = sin(264*2*M_PI*Time);
        Sint16 Value = 1000*ValueF;
        
        Samples[I].L = Value;
        Samples[I].R = Value;
    }
    
    Audio->SamplesDone += SampleCount;
}

int main(int, char **) {
    if(SDL_Init(SDL_INIT_VIDEO)) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return -1;
    }
    
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    //SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_FRAMEBUFFER_SRGB_CAPABLE, 1);
    
    int WindowWidth = 192*6;
    int WindowHeight = 108*6;
    SDL_Window *Window = SDL_CreateWindow("SDL Template", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WindowWidth, WindowHeight, SDL_WINDOW_OPENGL);
    if(Window == 0) {
        fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        return -1;
    }
    
    SDL_GLContext Context = SDL_GL_CreateContext(Window);
    if(Context == 0) {
        fprintf(stderr, "SDL_GL_CreateContext: %s\n", SDL_GetError());
        return -1;
    }
    
    if(SDL_GL_SetSwapInterval(1)) {
        fprintf(stderr, "SDL_GL_SetSwapInterval: %s\n", SDL_GetError());
    }
    
    if(gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress) == 0) {
        fprintf(stderr, "glad gl load failed\n");
        return -1;
    }
    
    GLuint Shader;
    if(LoadShader(&Shader, "default.vs", "default.fs")) {
        fprintf(stderr, "shader compilation failed\n");
        return -1;
    }
    
    GLuint VAO, VBO;
    glGenBuffers(1, &VBO);
    glGenVertexArrays(1, &VAO);
    
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(MeshData), MeshData, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (void *)(0));
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (void *)(3*sizeof(GLfloat)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    glCheck(glEnable(GL_FRAMEBUFFER_SRGB));
    
    GLfloat ModelPosition[2] = {0};
    
    float DeltaTime = 0.0;
    Uint32 LastTicks = SDL_GetTicks();
    float Angle = 0.0f;
    
    float LastMouseX = 0.0f;
    float LastMouseY = 0.0f;
    float MouseX = 0.0f;
    float MouseY = 0.0f;
    
    float AngleX = 0.0f;
    float AngleY = 0.0f;
    
    int IsButtonDown = 0;
    
    int Running = 1;
    while (Running) {
        SDL_Event Event;
        LastMouseX = MouseX;
        LastMouseY = MouseY;
        while (SDL_PollEvent(&Event)) {
            switch (Event.type) {
            case SDL_QUIT:
                Running = 0;
                break;
            case SDL_MOUSEBUTTONDOWN:
                ModelPosition[0] = (2.0f*Event.button.x)/WindowWidth - 1.0f;
                ModelPosition[1] = (-2.0f*Event.button.y)/WindowHeight + 1.0f;
                IsButtonDown = 1;
                break;
            case SDL_MOUSEBUTTONUP:
                IsButtonDown = 0;
                break;
            case SDL_MOUSEMOTION:
                MouseX = Event.motion.x;
                MouseY = Event.motion.y;
                break;
            default:
                break;
            }
        }
        
        float DeltaMouseX = MouseX - LastMouseX;
        float DeltaMouseY = MouseY - LastMouseY;
        
        if(IsButtonDown) {
            AngleX += 0.004f*DeltaMouseY;
            AngleY += 0.004f*DeltaMouseX;
        }
        
        std::cout << DeltaMouseX << std::endl;
        
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glCheck(glClear(GL_COLOR_BUFFER_BIT));
        
        Angle += DeltaTime;
        m3 R = RotationX(AngleX)*RotationY(AngleY);
        
        glCheck(glUseProgram(Shader));
        //glCheck(glUniform2fv(glGetUniformLocation(Shader, "ModelP"), 1, ModelPosition));
        glCheck(glUniformMatrix3fv(glGetUniformLocation(Shader, "R"), 1, GL_FALSE, R.data()));
        
        glCheck(glBindVertexArray(VAO));
        glCheck(glDrawArrays(GL_TRIANGLES, 0, 6));
        glCheck(glBindVertexArray(0));
        
        glCheck(glUseProgram(0));

        SDL_GL_SwapWindow(Window);
        
        Uint32 CurrTicks = SDL_GetTicks();
        DeltaTime = ((CurrTicks - LastTicks)/1000.0f);
        std::cout << DeltaTime << std::endl;
        LastTicks = CurrTicks;
    }
    
    SDL_GL_DeleteContext(Context);
    SDL_DestroyWindow(Window);
    SDL_Quit();

    return 0;
}
