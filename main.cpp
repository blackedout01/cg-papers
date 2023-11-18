#include <stdio.h>
#include <math.h>
#include "Eigen/Dense"
#include "SDL.h"
#include "glad/glad.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

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
typedef Eigen::Vector4f v4;
typedef Eigen::Matrix2f m2;
typedef Eigen::Matrix3f m3;
typedef Eigen::Matrix4f m4;

#include <stdint.h>
typedef uint32_t u32;
typedef uint64_t u64;

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

static m4 M4(m3 M, v3 T=v3(0.0, 0.0, 0.0)) {
    v4 X(1.0f, 0.0f, 0.0f, 0.0f);
    v4 Y(0.0f, 1.0f, 0.0f, 0.0f);
    v4 Z(0.0f, 0.0f, 1.0f, 0.0f);
    v4 W(T[0], T[1], T[2], 1.0f);
    
    m4 Result;
    Result << X, Y, Z, W;
    
    Result(0, 0) = M(0, 0);
    Result(0, 1) = M(0, 1);
    Result(0, 2) = M(0, 2);
    Result(1, 0) = M(1, 0);
    Result(1, 1) = M(1, 1);
    Result(1, 2) = M(1, 2);
    Result(2, 0) = M(2, 0);
    Result(2, 1) = M(2, 1);
    Result(2, 2) = M(2, 2);
    return Result;
}

static m4 Translation(v3 T) {
    m4 Result = m4::Identity();
    Result(0, 3) = T[0];
    Result(1, 3) = T[1];
    Result(2, 3) = T[2];
    return Result;
}

static m3 Scale(float S) {
    m3 Result = S*m3::Identity();
    return Result;
}

static m4 Perspective(float FoVY, float WoH, float N, float F) {
    m4 Result;
    
    float T = tanf(FoVY/2.0f);
    float SX = 1.0f/(T*WoH);
    float SY = 1.0f/T;
    float A = (N + F)/(N - F);
    float B = 2.0f*N*F/(N - F);
    
    Result <<
    SX,   0.0f,  0.0f, 0.0f,
    0.0f,   SY,  0.0f, 0.0f,
    0.0f, 0.0f,     A,    B,
    0.0f, 0.0f, -1.0f, 0.0f;
    return Result;
}

static m4 Orthogonal(float L, float R, float B, float T, float N, float F) {
    m4 Result;
    
    Result <<
    2.0f/(R - L), 0.0f, 0.0f, (L + R)/(L - R),
    0.0f, 2.0f/(T - B), 0.0f, (B + T)/(B - T),
    0.0f, 0.0f, -2.0/(F - N), (F + N)/(N - F),
    0.0f, 0.0f, 0.0f, 1.0f;
    return Result;
}

struct vertex {
    v3 Position;
    v3 Normal;
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

#if 0
static vertex MeshData[] = {
    { { 0.5f,  0.5f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
    { { -0.5f,  0.5f, 0.0f }, { 1.0f, 1.0f, 0.0f } },
    { { -0.5f, -0.5f, 0.0f }, { 0.0f, 1.0f, 0.0f } },
    
    { { 0.5f,  0.5f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
    { { 0.5f, -0.5f, 0.0f }, { 0.0f, 0.0f, 1.0f } },
    { { -0.5f, -0.5f, 0.0f }, { 0.0f, 1.0f, 0.0f } },
    
    { { 5.0f,  -1.0f, 5.0f }, { 1.0f, 0.0f, 0.0f } },
    { { -5.0f, -1.0f, 5.0f }, { 1.0f, 1.0f, 0.0f } },
    { { -5.0f, -1.0f, -5.0f }, { 0.0f, 1.0f, 0.0f } },
    
    { { 5.0f,  -1.0f, 5.0f }, { 1.0f, 0.0f, 0.0f } },
    { { 5.0f, -1.0f, -5.0f }, { 0.0f, 0.0f, 1.0f } },
    { { -5.0f, -1.0f, -5.0f }, { 0.0f, 1.0f, 0.0f } },
};
#endif

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

struct mesh_triangle {
    u64 PositionIndices[3];
    u64 NormalIndices[3];
};

struct mesh {
    v3 *Positions;
    v3 *Normals;
    
    mesh_triangle *Triangles;
    
    u64 PositionCount;
    u64 NormalCount;
    
    u64 TriangleCount;
    
    GLuint VAO, VBO;
};

static void MeshToOpenGL(mesh *Mesh) {
    glGenBuffers(1, &Mesh->VBO);
    glGenVertexArrays(1, &Mesh->VAO);
    
    glBindVertexArray(Mesh->VAO);
    glBindBuffer(GL_ARRAY_BUFFER, Mesh->VBO);
    
    u64 ByteCount = Mesh->TriangleCount*3*sizeof(vertex);
    glBufferData(GL_ARRAY_BUFFER, ByteCount, 0, GL_STATIC_DRAW);
    
    vertex *Vertices = (vertex *)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for(u64 I = 0; I < Mesh->TriangleCount; ++I) {
        mesh_triangle Triangle = Mesh->Triangles[I];
        for(u64 J = 0; J < 3; ++J) {
            vertex Vertex;
            Vertex.Position = Mesh->Positions[Triangle.PositionIndices[J]];
            Vertex.Normal = Mesh->Normals[Triangle.NormalIndices[J]];
            
            *Vertices++ = Vertex;
        }
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (void *)(0));
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (void *)(3*sizeof(GLfloat)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

static void RenderMesh(mesh *Mesh, m4 MatM, m3 MatN, v4 Color, GLuint Shader) {
    glCheck(glUniformMatrix4fv(glGetUniformLocation(Shader, "MatM"), 1, GL_FALSE, MatM.data()));
    glCheck(glUniformMatrix3fv(glGetUniformLocation(Shader, "MatN"), 1, GL_FALSE, MatN.data()));
    glCheck(glUniform4fv(glGetUniformLocation(Shader, "Color"), 1, Color.data()));
    
    glCheck(glBindVertexArray(Mesh->VAO));
    glCheck(glDrawArrays(GL_TRIANGLES, 0, 3*Mesh->TriangleCount));
    glCheck(glBindVertexArray(0));
}

static int LoadMesh(mesh *OutMesh, const char *FileName) {
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./"; // Path to material files
    
    tinyobj::ObjReader reader;
    
    if (!reader.ParseFromFile(FileName, reader_config)) {
        if (!reader.Error().empty()) {
          std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }
    
    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
        return 1;
    }
    
    auto& Attrib = reader.GetAttrib();
    
    const std::vector<tinyobj::shape_t> &Shapes = reader.GetShapes();
    if(Shapes.size() > 1) {
        return 1;
    }
    std::cout << "Loading file " << FileName << " with shape " << Shapes[0].name << " ..." << std::endl;
    
    for(size_t I = 0; I < Shapes[0].mesh.num_face_vertices.size(); ++I) {
        if(Shapes[0].mesh.num_face_vertices[I] != 3) {
            std::cout << "Only triangles supported" << std::endl;
            return 1;
        }
    }
    
    mesh Mesh = {0};
    Mesh.PositionCount = Attrib.vertices.size()/3;
    Mesh.NormalCount = Attrib.normals.size()/3;
    Mesh.TriangleCount = Shapes[0].mesh.num_face_vertices.size();
    
    Mesh.Positions = (v3 *)malloc(sizeof(v3)*Mesh.PositionCount);
    Mesh.Normals = (v3 *)malloc(sizeof(v3)*Mesh.NormalCount);
    Mesh.Triangles = (mesh_triangle *)malloc(sizeof(mesh_triangle)*Mesh.TriangleCount);
    
    for(u64 I = 0; I < Mesh.PositionCount; ++I) {
        v3 Position(Attrib.vertices[3*I + 0],
                   Attrib.vertices[3*I + 1],
                   Attrib.vertices[3*I + 2]);
        Mesh.Positions[I] = Position;
    }
    
    for(u64 I = 0; I < Mesh.NormalCount; ++I) {
        v3 Normal(Attrib.normals[3*I + 0],
                   Attrib.normals[3*I + 1],
                   Attrib.normals[3*I + 2]);
        Mesh.Normals[I] = Normal;
    }
    
    for(size_t TriangleIndex = 0; TriangleIndex < Mesh.TriangleCount; ++TriangleIndex) {
        mesh_triangle Triangle = {0};
        
        for(size_t I = 0; I < 3; ++I) {
            tinyobj::index_t ObjIndex = Shapes[0].mesh.indices[3*TriangleIndex + I];
            
            Triangle.PositionIndices[I] = ObjIndex.vertex_index;
            Triangle.NormalIndices[I] = ObjIndex.normal_index;
        }
        
        Mesh.Triangles[TriangleIndex] = Triangle;
    }
    
    *OutMesh = Mesh;
    
    return 0;
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

    GLuint LightShader;
    if(LoadShader(&LightShader, "light.vs", "light.fs")) {
        fprintf(stderr, "light shader compilation failed\n");
        return -1;
    }

    GLuint DepthTex;
    int DepthTexSize = 1024;
    glCheck(glGenTextures(1, &DepthTex));
    glCheck(glBindTexture(GL_TEXTURE_2D, DepthTex));
    glCheck(glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, DepthTexSize, DepthTexSize, 0,  GL_DEPTH_COMPONENT,  GL_FLOAT, 0));
    glCheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
    glCheck(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
    glCheck(glBindTexture(GL_TEXTURE_2D, 0));

    GLuint LightFBO;
    glCheck(glGenFramebuffers(1, &LightFBO));
    glCheck(glBindFramebuffer(GL_FRAMEBUFFER, LightFBO));
    glCheck(glDrawBuffer(GL_NONE));
    glCheck(glReadBuffer(GL_NONE));
    glCheck(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, DepthTex, 0));
    glCheck(glBindFramebuffer(GL_FRAMEBUFFER, 0));
#if 0
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
#endif
    
    glCheck(glEnable(GL_FRAMEBUFFER_SRGB));
    glEnable(GL_DEPTH_TEST);
    
    GLfloat ModelPosition[2] = {0};
    
    mesh Meshes[2];
    LoadMesh(Meshes + 0, "cube.obj");
    LoadMesh(Meshes + 1, "plane.obj");
    MeshToOpenGL(Meshes + 0);
    MeshToOpenGL(Meshes + 1);
    
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
    float Zoom = 0.4f;
    
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
            case SDL_MOUSEWHEEL:
                if(Event.wheel.y < 0) {
                    Zoom *= 1.1f;
                } else if(Event.wheel.y > 0) {
                    Zoom /= 1.1f;
                }
                break;
            default:
                break;
            }
        }
        
        float DeltaMouseX = MouseX - LastMouseX;
        float DeltaMouseY = MouseY - LastMouseY;
        
        if(IsButtonDown) {
            AngleX -= 0.004f*DeltaMouseY;
            AngleY -= 0.004f*DeltaMouseX;
        }
        
        m4 CubeMatM = Translation(v3(0.0f, 0.0f, 0.0f))*M4(RotationX(Angle));
        m3 CubeMatN = RotationX(Angle);
        m4 PlaneMatM = Translation(v3(0.0f, -1.2f, 0.0f))*M4(Scale(10.0f));
        m3 PlaneMatN = m3::Identity();

        float EdgeSize = 20.0f;
        float LightPhi = 0.5f;
        float LightTheta = -0.9f;
        m3 LightRotation = RotationY(LightPhi)*RotationX(LightTheta);
        v3 LightDir = LightRotation*v3(0.0f, 0.0f, -1.0f);
        m4 LightV = M4(RotationX(-LightTheta)*RotationY(-LightPhi));
        m4 LightP = Orthogonal(-EdgeSize, EdgeSize, -EdgeSize, EdgeSize, -EdgeSize, EdgeSize);
        m4 LightPV = LightP*LightV;

        glCheck(glBindFramebuffer(GL_FRAMEBUFFER, LightFBO));
        glCheck(glClear(GL_DEPTH_BUFFER_BIT));
        glViewport(0, 0, DepthTexSize, DepthTexSize);
        glCheck(glUseProgram(LightShader));
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);

        glCheck(glUniformMatrix4fv(glGetUniformLocation(LightShader, "MatPV"), 1, GL_FALSE, LightPV.data()));
        RenderMesh(Meshes + 0, CubeMatM, CubeMatN, v4(1.0f, 1.0f, 1.0f, 1.0f), LightShader);
        RenderMesh(Meshes + 1, PlaneMatM, PlaneMatN, v4(0.04f, 0.04f, 0.06f, 1.0f), LightShader);

        glCheck(glBindFramebuffer(GL_FRAMEBUFFER, 0));

        glClearColor(0.68f, 0.76f, 0.99f, 1.0f);
        //glClearDepth(1.0f);
        glCheck(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
        glViewport(0, 0, WindowWidth, WindowHeight);

        Angle += DeltaTime;
        m3 R = RotationX(AngleX)*RotationY(AngleY);
        m4 R4 = M4(Scale(0.6));//M4(R, v3(0.1, 0.0, 0.0));
        
        float CameraRadius = 6.0f;
        m4 C = M4(Scale(Zoom))*M4(RotationY(AngleY))*M4(RotationX(AngleX))*Translation(v3(0.0f, 0.0f, CameraRadius));
        m4 V = Translation(v3(0.0f, 0.0f, -CameraRadius))*M4(RotationX(-AngleX))*M4(RotationY(-AngleY))*M4(Scale(1.0f/Zoom));
        m4 P = Perspective(1.39626, WindowWidth/(float)WindowHeight, 0.1f, 100.0f);
        
        glCheck(glUseProgram(Shader));
        glCullFace(GL_BACK);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, DepthTex);
        glCheck(glUniform1i(glGetUniformLocation(Shader, "LightDepthTex"), 0));

        //glCheck(glUniform2fv(glGetUniformLocation(Shader, "ModelP"), 1, ModelPosition));
        glCheck(glUniformMatrix4fv(glGetUniformLocation(Shader, "R"), 1, GL_FALSE, R4.data()));
        glCheck(glUniformMatrix4fv(glGetUniformLocation(Shader, "MatV"), 1, GL_FALSE, V.data()));
        glCheck(glUniformMatrix4fv(glGetUniformLocation(Shader, "MatP"), 1, GL_FALSE, P.data()));
        glCheck(glUniformMatrix4fv(glGetUniformLocation(Shader, "LightPV"), 1, GL_FALSE, LightPV.data()));
        
        glCheck(glUniform3fv(glGetUniformLocation(Shader, "LightDir"), 1, LightDir.data()));
        
        RenderMesh(Meshes + 0, CubeMatM, CubeMatN, v4(1.0f, 1.0f, 1.0f, 1.0f), Shader);
        RenderMesh(Meshes + 1, PlaneMatM, PlaneMatN, v4(0.04f, 0.04f, 0.06f, 1.0f), Shader);
        
        glCheck(glUseProgram(0));

        SDL_GL_SwapWindow(Window);
        
        Uint32 CurrTicks = SDL_GetTicks();
        DeltaTime = ((CurrTicks - LastTicks)/1000.0f);
        //std::cout << DeltaTime << std::endl;
        LastTicks = CurrTicks;
    }
    
    SDL_GL_DeleteContext(Context);
    SDL_DestroyWindow(Window);
    SDL_Quit();

    return 0;
}
