/*
 * g++ FrAnK.cpp -o frank -luvc -lpng -lzip -pthread -ljpeg `pkg-config --cflags --libs opencv4`
 */

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <regex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <libuvc/libuvc.h>
#include <fstream>
#include <png.h>
#include <zip.h>
#include <jpeglib.h>
//#include <random>
#include <mutex>
#include <ctime>
#include <iomanip>

using namespace std;
using namespace cv;

ofstream logFile;

constexpr int WIDTH = 16;
constexpr int HEIGHT = 12;

// 🔹 Nowa globalna zmienna na znacznik czasu ramki
uint64_t irFrameTimestamp = 0;

// Deklaracje funkcji
bool readTemperatureFrame(int fd, vector<float>& temperatures, float& minTemp, float& maxTemp);
Mat generateThermalImage(const vector<float>& temperatures, float minTemp, float maxTemp);

std::mutex g_time_mutex;
std::chrono::steady_clock::time_point g_last_frame_time = std::chrono::steady_clock::now();

std::atomic<bool> g_camera_error{false};

// Globalny czas wywołania
std::chrono::steady_clock::time_point g_last_update_time = std::chrono::steady_clock::now();

// Globalna zmienna czasu w ms, początkowa losowa wartość
uint64_t g_time_ms = 0;

// Punkt startowy do odmierzania czasu od początku programu
auto g_start_time = std::chrono::steady_clock::now();

// Załóżmy, że g_start_time to std::chrono::steady_clock::time_point
extern std::chrono::steady_clock::time_point g_start_time;



// Funkcja do pobrania aktualnego czasu w formacie RR.MM.DD_GG.MM_
std::string getDateTimePrefix() {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::setfill('0')
        << std::setw(2) << (tm.tm_year % 100) << "."      // RR (ostatnie dwie cyfry roku)
        << std::setw(2) << (tm.tm_mon + 1) << "."         // MM (miesiąc od 0)
        << std::setw(2) << tm.tm_mday << "_"               // DD (dzień)
        << std::setw(2) << tm.tm_hour << "."
        << std::setw(2) << tm.tm_min << "_";

    return oss.str();
}

//################## uvc

// Wylicza obecny czas ramki (ms) na podstawie startowego czasu i czasu który minął
uint64_t uvc_get_current_frame_time() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_update_time).count();
    return g_time_ms + elapsed;
}

// Dekodowanie MJPEG do RGB
bool uvc_decode_mjpeg_to_rgb(const unsigned char* mjpeg_data, size_t mjpeg_size, unsigned char* rgb_buf, int width, int height) {
    jpeg_decompress_struct cinfo;
    jpeg_error_mgr jerr;
    
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, mjpeg_data, mjpeg_size);
    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }
    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);

    if (cinfo.output_width != width || cinfo.output_height != height) {
        jpeg_finish_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    unsigned char* rowptr[1];
    while (cinfo.output_scanline < cinfo.output_height) {
        rowptr[0] = rgb_buf + cinfo.output_scanline * width * 3;
        jpeg_read_scanlines(&cinfo, rowptr, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return true;
}

// Kontekst aplikacji
struct AppContext {
    uvc_context_t* ctx = nullptr;
    uvc_device_t* dev = nullptr;
    uvc_device_handle_t* devh = nullptr;
    uvc_stream_ctrl_t ctrl{};
    zip_t* zip = nullptr;
    std::string zip_path;
    std::atomic<bool> should_save_frame{false};
};

// Kodowanie PNG
bool uvc_encode_png(int width, int height, const unsigned char* rgb, std::vector<unsigned char>& out) {
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) return false;

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, nullptr);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return false;
    }

    struct PngMem {
        std::vector<unsigned char>* buffer;
    } mem{&out};

    png_set_write_fn(png_ptr, &mem,
        [](png_structp png_ptr, png_bytep data, png_size_t length) {
            auto* mem = (PngMem*)png_get_io_ptr(png_ptr);
            mem->buffer->insert(mem->buffer->end(), data, data + length);
        },
        [](png_structp) {}
    );

    png_set_IHDR(png_ptr, info_ptr, width, height,
                 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, info_ptr);

    std::vector<png_bytep> rows(height);
    for (int y = 0; y < height; ++y)
        rows[y] = const_cast<png_bytep>(rgb + y * width * 3);  // png_write_image wymaga png_bytep (czyli unsigned char*)

    png_write_image(png_ptr, rows.data());
    png_write_end(png_ptr, nullptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return true;
}

// 1. Pobierz i zdekompresuj obraz RGB z MJPEG
bool uvc_get_rgb_from_frame(const uvc_frame_t* frame, std::vector<unsigned char>& rgb_buf) {
    rgb_buf.resize(frame->width * frame->height * 3);
    if (!uvc_decode_mjpeg_to_rgb((unsigned char*)frame->data, frame->data_bytes, rgb_buf.data(), frame->width, frame->height)) {
        std::cerr << "Błąd dekodowania MJPEG do RGB\n";
        g_camera_error = true;
        return false;
    }
    return true;
}

// 2. Zapisz obraz RGB do PNG w archiwum ZIP
bool uvc_save_frame_to_zip(AppContext* ctx, const std::vector<unsigned char>& rgb_buf, int width, int height, uint64_t start_frame_time_ms) {
    uint64_t current_frame_time = uvc_get_current_frame_time();

    std::vector<unsigned char> png_data;
    if (!uvc_encode_png(width, height, rgb_buf.data(), png_data)) {
        std::cerr << "Błąd kodowania PNG\n";
        g_camera_error = true;
        return false;
    }

    char filename[64];
    snprintf(filename, sizeof(filename), "frame_%llu.png", (unsigned long long)current_frame_time);

    zip_source_t* source = zip_source_buffer(ctx->zip, png_data.data(), png_data.size(), 0);
    if (!source) {
        std::cerr << "Błąd tworzenia źródła ZIP\n";
        return false;
    }

    if (zip_file_add(ctx->zip, filename, source, ZIP_FL_OVERWRITE) < 0) {
        zip_source_free(source);
        std::cerr << "Błąd dodawania pliku do ZIP\n";
        return false;
    }

    if (zip_close(ctx->zip) < 0) {
        std::cerr << "Błąd zamykania pliku ZIP: " << zip_strerror(ctx->zip) << "\n";
        return false;
    }
    ctx->zip = nullptr;

    int err = 0;
    ctx->zip = zip_open(ctx->zip_path.c_str(), ZIP_CHECKCONS, &err);
    if (!ctx->zip) {
        zip_error_t ziperr;
        zip_error_init_with_code(&ziperr, err);
        std::cerr << "Błąd otwierania pliku ZIP: " << zip_error_strerror(&ziperr) << "\n";
        zip_error_fini(&ziperr);
        return false;
    }

    std::cout << "Zapisano " << filename << " i odświeżono ZIP\n";
    return true;
}

// 3. Wyświetl obraz RGB skalowany do maks. 800x600
void uvc_show_scaled_image(const std::vector<unsigned char>& rgb_buf, int width, int height) {
    cv::Mat img(height, width, CV_8UC3, const_cast<unsigned char*>(rgb_buf.data()));

    cv::Mat img_bgr;
    cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);

    const int max_width = 800;
    const int max_height = 600;

    int new_width = img_bgr.cols;
    int new_height = img_bgr.rows;

    double scale_w = static_cast<double>(max_width) / new_width;
    double scale_h = static_cast<double>(max_height) / new_height;
    double scale = std::min(scale_w, scale_h);

    if (scale < 1.0) {
        new_width = static_cast<int>(new_width * scale);
        new_height = static_cast<int>(new_height * scale);
        cv::Mat img_resized;
        cv::resize(img_bgr, img_resized, cv::Size(new_width, new_height));
        img_bgr = img_resized;
    }

    cv::imshow("Ostatnia klatka", img_bgr);
    cv::waitKey(10);
}

void uvc_frame_callback(uvc_frame_t* frame, void* ptr) {
    auto* ctx = reinterpret_cast<AppContext*>(ptr);
    if (!ctx->should_save_frame) return;

    {
        std::lock_guard<std::mutex> lock(g_time_mutex);
        g_last_frame_time = std::chrono::steady_clock::now();
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_start_time).count();
    uint64_t frame_time = g_time_ms + elapsed_ms;

    std::vector<unsigned char> rgb_buf;
    if (!uvc_get_rgb_from_frame(frame, rgb_buf))
        return;

    if (!uvc_save_frame_to_zip(ctx, rgb_buf, frame->width, frame->height, frame_time))
        return;

    uvc_show_scaled_image(rgb_buf, frame->width, frame->height);
    waitKey(10);

    ctx->should_save_frame = false;
}

// Inicjalizacja i konfiguracja 
bool uvc_initialize(AppContext& ctx, const std::string& zip_file_path) {
    cv::namedWindow("Ostatnia klatka", cv::WINDOW_AUTOSIZE);
    cv::waitKey(100);

    g_last_frame_time = std::chrono::steady_clock::now();

    ctx.zip_path = zip_file_path;

    // Inicjalizacja losowego startu czasu
    //std::random_device rd;
    //g_time_ms = rd() % 1000000;  // np. max 1 mln ms (1000s) jako offset startowy

    std::cout << "Inicjalizacja kontekstu UVC...\n";
    uvc_error_t res = uvc_init(&ctx.ctx, nullptr);
    if (res < 0) {
        std::cerr << "Błąd: uvc_init nie powiodło się: " << uvc_strerror(res) << "\n";
        return false;
    }

    std::cout << "Szukanie urządzenia UVC...\n";
    res = uvc_find_device(ctx.ctx, &ctx.dev, 0, 0, nullptr);
    if (res < 0) {
        std::cerr << "Błąd: uvc_find_device nie powiodło się: " << uvc_strerror(res) << "\n";
        return false;
    }

    std::cout << "Otwarcie urządzenia...\n";
    res = uvc_open(ctx.dev, &ctx.devh);
    if (res < 0) {
        std::cerr << "Błąd: uvc_open nie powiodło się: " << uvc_strerror(res) << "\n";
        return false;
    }

    uvc_device_descriptor_t* desc;
    if (uvc_get_device_descriptor(ctx.dev, &desc) == UVC_SUCCESS) {
        std::cout << "Znaleziono urządzenie:\n";
        if (desc->manufacturer) std::cout << "  Producent: " << desc->manufacturer << "\n";
        if (desc->product)     std::cout << "  Model: " << desc->product << "\n";
        if (desc->serialNumber) std::cout << "  S/N: " << desc->serialNumber << "\n";
        uvc_free_device_descriptor(desc);
    }

    std::cout << "Konfiguracja streamu: 1920x1080 @ 25 FPS, MJPEG...\n";
    res = uvc_get_stream_ctrl_format_size(
        ctx.devh,
        &ctx.ctrl,
        UVC_FRAME_FORMAT_MJPEG,
        1920,
        1080,
        25
    );

    if (res < 0) {
        std::cerr << "Błąd: uvc_get_stream_ctrl_format_size nie powiodło się: " << uvc_strerror(res) << "\n";
        return false;
    }

    std::cout << "Otwieranie pliku ZIP: " << ctx.zip_path << "\n";
    int errcode = 0;
    ctx.zip = zip_open(ctx.zip_path.c_str(), ZIP_CREATE | ZIP_TRUNCATE, &errcode);
    if (!ctx.zip) {
        zip_error_t ziperr;
        zip_error_init_with_code(&ziperr, errcode);
        std::cerr << "Błąd: nie można otworzyć pliku ZIP: " << zip_error_strerror(&ziperr) << "\n";
        zip_error_fini(&ziperr);
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "Inicjalizacja zakończona sukcesem.\n";
    return true;
}

// Sprzątanie
void uvc_cleanup(AppContext& ctx) {
    if (ctx.devh) {
        uvc_stop_streaming(ctx.devh);
    }

    if (ctx.zip) {
        zip_close(ctx.zip);
        ctx.zip = nullptr;
    }

    if (ctx.devh) {
        uvc_close(ctx.devh);
        ctx.devh = nullptr;
    }

    if (ctx.dev) {
        uvc_unref_device(ctx.dev);
        ctx.dev = nullptr;
    }

    if (ctx.ctx) {
        uvc_exit(ctx.ctx);
        ctx.ctx = nullptr;
    }
}

bool uvc_reinitialize_camera(AppContext& ctx) {
    std::cerr << "Wykryto błąd kamery, próba ponownej inicjalizacji...\n";

    // Zatrzymaj streaming i zamknij urządzenie
    if (ctx.devh) {
        uvc_stop_streaming(ctx.devh);
        uvc_close(ctx.devh);
        ctx.devh = nullptr;
    }
    if (ctx.ctx) {
        uvc_exit(ctx.ctx);
        ctx.ctx = nullptr;
    }

    // Krótkie opóźnienie, żeby sprzęt "ochłonął"
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Spróbuj ponownie zainicjować
    if (!uvc_initialize(ctx, ctx.zip_path.c_str())) {
        std::cerr << "Ponowna inicjalizacja kamery nie powiodła się.\n";
        return false;
    }

    // Spróbuj ponownie rozpocząć streaming
    if (uvc_start_streaming(ctx.devh, &ctx.ctrl, uvc_frame_callback, &ctx, 0) != UVC_SUCCESS) {
        std::cerr << "Nie udało się rozpocząć streamu po ponownej inicjalizacji.\n";
        return false;
    }

    // Reset flag i czas ostatniej ramki
    g_camera_error = false;
    {
        std::lock_guard<std::mutex> lock(g_time_mutex);
        g_last_frame_time = std::chrono::steady_clock::now();
    }

    std::cout << "Ponowna inicjalizacja kamery zakończona sukcesem.\n";
    return true;
}

void uvc_update_frame_time(int delta_ms) {
    g_time_ms = delta_ms;
    g_last_update_time = std::chrono::steady_clock::now();

    std::cout << "Zaktualizowany czas bazowy: " << g_time_ms << " ms\n";
}

//################## koniec uvc

enum class FrameType {
    None,
    IR,
    Scale,
    Motor,
    Other
};

struct Frame {
    FrameType type = FrameType::None;
    uint64_t timestamp = 0;
    vector<string> dataLines;
};

float weightValue;
int motorPos = -1;

int setupSerialPort(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        cerr << "Nie można otworzyć portu " << device << ": " << strerror(errno) << "\n";
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        cerr << "Błąd tcgetattr: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 1;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "Błąd tcsetattr: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    return fd;
}

bool readLine(int fd, string& line) {
    line.clear();
    char c;
    while (true) {
        int n = read(fd, &c, 1);
        if (n < 0) {
            cerr << "Błąd odczytu: " << strerror(errno) << "\n";
            return false;
        } else if (n == 0) {
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        if (c == '\n') break;
        if (c != '\r') line += c;
    }

    if (logFile.is_open()) {
        logFile << line << '\n';
    }else{
        logFile << line << '\n';
        cout << "logfile not opened!" <<endl;
    }

    return true;
}

int readFrame(int fd, Frame& frame) {
    string line;

    regex irStartRegex(R"(-- IR Start -- (\d+))");
    regex scaleStartRegex(R"(-- Scale Start -- (\d+))");
    regex motorStartRegex(R"(-- Motor Start -- (\d+))");
    smatch match;

    frame.type = FrameType::None;
    frame.timestamp = 0;
    frame.dataLines.clear();

    // Szukamy znacznika startu
    while (readLine(fd, line)) {
        if (regex_search(line, match, irStartRegex)) {
            frame.type = FrameType::IR;
            frame.timestamp = stoull(match[1]);
            break;
        } else if (regex_search(line, match, scaleStartRegex)) {
            frame.type = FrameType::Scale;
            frame.timestamp = stoull(match[1]);
            break;
        } else if (regex_search(line, match, motorStartRegex)) {
            frame.type = FrameType::Motor;
            frame.timestamp = stoull(match[1]);
            break;
        } else if (line.find("--") != string::npos) {
            // Możesz tu dodać wykrywanie innych typów ramek, np.:
            frame.type = FrameType::Other;
            frame.timestamp = 0;
            break;
        }
    }

    if (frame.type == FrameType::None) {
        // Nie znaleziono znacznika startu, brak ramki
        return 0;
    }
    
    uvc_update_frame_time(frame.timestamp);

    // Dobieramy znacznik stop w zależności od typu
    string stopMarker;
    switch (frame.type) {
        case FrameType::IR: stopMarker = "-- IR Stop --"; break;
        case FrameType::Scale: stopMarker = "-- Scale Stop --"; break;
        case FrameType::Motor: stopMarker = "-- Motor Stop --"; break;
        case FrameType::Other: stopMarker = "--"; break; // Załóżmy, że inne też mają jakiś stop
        default: stopMarker = ""; break;
    }

    // Czytamy linie aż do stop
    while (readLine(fd, line)) {
        if (!stopMarker.empty() && line.find(stopMarker) != string::npos)
            break;

        size_t pos = line.find("->");
        if (pos != string::npos) {
            line = line.substr(pos + 2);
        }

        // Przytnij białe znaki
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (!line.empty()) {
            frame.dataLines.push_back(line);
        }
    }

    return 1;
}

bool parseIRData(const Frame& frame, vector<float>& temperatures, float& minTemp, float& maxTemp) {
    if (frame.dataLines.size() != HEIGHT) {
        cerr << "Błąd: oczekiwano " << HEIGHT << " linii danych, otrzymano " << frame.dataLines.size() << "\n";
        return false;
    }

    temperatures.clear();
    minTemp = 1000.f;
    maxTemp = -1000.f;

    for (int i = 0; i < HEIGHT; ++i) {
        istringstream iss(frame.dataLines[i]);
        for (int j = 0; j < WIDTH; ++j) {
            float val;
            if (!(iss >> val)) {
                cerr << "Błąd parsowania danych w linii " << i << ": " << frame.dataLines[i] << "\n";
                return false;
            }
            temperatures.push_back(val);
            if (val < minTemp) minTemp = val;
            if (val > maxTemp) maxTemp = val;
        }
    }

    if (temperatures.size() != WIDTH * HEIGHT) {
        cerr << "Błąd: nieprawidłowa liczba danych: " << temperatures.size() << "\n";
        return false;
    }

    return true;
}

bool parseScaleData(const Frame& frame, float& weight) {
    weight = 0.0f;  // domyślna wartość lub zerowa waga
    for (const auto& line : frame.dataLines) {
        //cout << "  " << line << "\n";

        // Szukamy linii z wagą, np. "Weight: 0.00"
        if (line.find("Weight:") == 0) {
            // Parsujemy liczbę po "Weight:"
            istringstream iss(line.substr(7)); // pomijamy "Weight:"
            float val;
            if (iss >> val) {
                weight = val;
                return true;  // znaleźliśmy i odczytaliśmy wagę
            } else {
                cerr << "Błąd parsowania wartości wagi w linii: " << line << "\n";
                return false;
            }
        }
    }

    cerr << "Nie znaleziono wartości wagi w danych Scale\n";
    return false; // brak wagi w liniach
}

bool parseMotorData(const Frame& frame, int& position) {
    for (const auto& line : frame.dataLines) {
        // Sprawdź, czy linia zaczyna się od "Position:"
        if (line.find("Position:") == 0) {
            // Wyciągnij substring po "Position:"
            string numberStr = line.substr(9); // "Position:" ma 9 znaków
            istringstream iss(numberStr);

            int val;
            if (iss >> val) {
                position = val;
                return true;  // udało się sparsować pozycję
            } else {
                cerr << "Błąd parsowania wartości pozycji w linii: " << line << "\n";
                return false;
            }
        }
    }

    cerr << "Nie znaleziono wartości pozycji w danych Motor\n";
    return false; // brak pozycji w danych
}

void parseOtherData(const Frame& frame) {
    cout << "Otrzymano ramkę innego typu:\n";
    cout << "Typ: " << static_cast<int>(frame.type) << ", linie danych:\n";
    for (const auto& line : frame.dataLines) {
        cout << "  " << line << "\n";
    }
}



Mat generateThermalImage(const vector<float>& temperatures, float minTemp, float maxTemp) {
    Mat thermalImg(HEIGHT, WIDTH, CV_32F, const_cast<float*>(temperatures.data()));

    Mat normalizedImg;
    thermalImg.convertTo(normalizedImg, CV_8U, 255.0 / (maxTemp - minTemp), -minTemp * 255.0 / (maxTemp - minTemp));

    int scale = 40;
    Mat displayImg;
    resize(normalizedImg, displayImg, Size(), scale, scale, INTER_NEAREST);

    Mat colorImg;
    applyColorMap(displayImg, colorImg, COLORMAP_JET);

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            int idx = y * WIDTH + x;
            float temp = temperatures[idx];

            Point textOrg(x * scale + 2, y * scale + scale - 4);

            char buf[16];
            snprintf(buf, sizeof(buf), "%.1f", temp);

            putText(colorImg, buf, textOrg, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 2, LINE_AA);
            putText(colorImg, buf, textOrg, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1, LINE_AA);
        }
    }

    return colorImg;
}

Mat generateInfoBar(int width, uint64_t timestamp, float weight, int motorPos) {
    int height = 40;  // wysokość paska w px
    Mat bar(height, width, CV_8UC3, Scalar(30, 30, 30)); // ciemnoszary pasek

    // Tworzymy napis z timestampem, wagą i pozycją silnika
    string infoStr = "Timestamp: " + to_string(timestamp) +
                     "  Weight: " + to_string(weight) +
                     "  Position: " + to_string(motorPos);

    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.6;//0.8
    int thickness = 2;
    int baseline = 0;

    Size textSize = getTextSize(infoStr, fontFace, fontScale, thickness, &baseline);

    Point textOrg((width - textSize.width) / 2, (height + textSize.height) / 2);

    // cień
    putText(bar, infoStr, textOrg + Point(1, 1), fontFace, fontScale, Scalar(0, 0, 0), thickness + 1, LINE_AA);
    // biały tekst
    putText(bar, infoStr, textOrg, fontFace, fontScale, Scalar(255, 255, 255), thickness, LINE_AA);

    return bar;
}


Mat concatImageAndInfoBar(const Mat& image, const Mat& timestampBar) {
    // Sprawdź zgodność szerokości
    if (image.cols != timestampBar.cols) {
        cerr << "Błąd: szerokość obrazu i paska timestampu musi być taka sama\n";
        return image;
    }

    Mat combined;
    vconcat(image, timestampBar, combined);
    return combined;
}


// Główna funkcja main
int main(int argc, char** argv) {
    if (argc < 2) {     //                  arg1        arg2 
        cerr << "Użycie: " << argv[0] << " <port tty> <nazwa>\n";
        return 1;
    }
    
    // Przypisanie do zmiennych
    const char* port = argv[1];
    
    // statyczny albo lokalny std::string trzymający połączony tekst
    std::string baseNameStr = getDateTimePrefix() + std::string(argv[2]);

    // baseName to const char* wskazujący na dane w baseNameStr
    const char* baseName = baseNameStr.c_str();

    std::string zipFile = std::string(baseName) + ".zip";
    //std::ofstream logFile;
    
    int fd = setupSerialPort(port);
    if (fd < 0) return 1;

    namedWindow("Thermal Image", WINDOW_AUTOSIZE);

    vector<float> temperatures;
    float minTemp, maxTemp;
    
    bool gotProperIR = 0;
    bool gotProperScale = 0;
    
    logFile.open((std::string(baseName) + ".txt").c_str(), ios::out | ios::app);
    if (!logFile.is_open()) {
        cerr << "Nie udało się otworzyć pliku do logowania: " << baseName << ".txt\n";
        return 1;
    }
    
    //############## uvc_
    AppContext ctx;
    
    uint64_t frame_time = 0; // np. globalny licznik ms do aktualizowania "obecnego" czasu.
    if (!uvc_initialize(ctx, zipFile)) {
        std::cerr << "Błąd inicjalizacji\n";
        uvc_cleanup(ctx);
        return 1;
    }
    
    if (uvc_start_streaming(ctx.devh, &ctx.ctrl, uvc_frame_callback, &ctx, 0) != UVC_SUCCESS) {
        std::cerr << "Nie udało się rozpocząć streamu\n";
        uvc_cleanup(ctx);
        uint64_t current_frame_time = uvc_get_current_frame_time();

        return 1;
    }
    
    std::cout << "Program uruchomiony. Zapis co 5 sekund z kamery usb.\n";
    
    auto next_save_time = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    g_last_frame_time = std::chrono::steady_clock::now();
    
    //############## koniec uvc_

    while (true) {
        
        //############# uvc_
        
        auto now = std::chrono::steady_clock::now();
        
        if (now >= next_save_time) {
            ctx.should_save_frame = true;
            next_save_time = now + std::chrono::seconds(5);
        }

        {
            std::lock_guard<std::mutex> lock(g_time_mutex);
            auto diff = now - g_last_frame_time;
            if (diff > std::chrono::seconds(10)) {
                std::cerr << "Brak ramek z kamery od 10 sekund — wykryto błąd!\n";
                g_camera_error = true;
            }
        }

        if (g_camera_error) {
            if (!uvc_reinitialize_camera(ctx)) {
                std::cerr << "Ponowna inicjalizacja nie powiodła się, ponawiam za 3 sekundy...\n";
                std::this_thread::sleep_for(std::chrono::seconds(3));
                continue;  // Spróbuj ponownie w pętli
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        //############ koniec uvc
        
        Frame frame;
        if (!readFrame(fd, frame)) {
            // brak pełnej ramki, można czekać lub robić coś innego
            
            continue;
        }else{
            //cout << "Pełna ramka.\n";
        }

        switch (frame.type) {
            case FrameType::IR:
                if (parseIRData(frame, temperatures, minTemp, maxTemp)) {
                    irFrameTimestamp = frame.timestamp;
                    //cout << "Dobra ramka IR.\n";
                    // Możesz generować obraz, wyświetlać itp.
                    gotProperIR = 1;
                } else {
                    cerr << "Błąd przetwarzania danych IR\n";
                    continue;
                }
                break;

            case FrameType::Scale:
                if (!parseScaleData(frame, weightValue)) {
                    cerr << "Błąd przetwarzania danych Scale\n";
                }else{   
                    gotProperScale = 1;
                }
                break;

            case FrameType::Motor:
                if (!parseMotorData(frame, motorPos)) {
                    cerr << "Błąd przetwarzania danych Motor\n";
                }else{   
                    //gotProperScale = 1;
                }
                break;

            case FrameType::Other:
                parseOtherData(frame);
                break;

            default:
                cerr << "Nieznany typ ramki\n";
                break;
        }

        if(gotProperIR + gotProperScale < 2){
            cout << "Dane niegotowe.\n";
            continue;
        }
        
        // Generujemy obraz termiczny
        Mat thermalImg = generateThermalImage(temperatures, minTemp, maxTemp);

        // Generujemy pasek timestampu
        Mat tsBar = generateInfoBar(thermalImg.cols, irFrameTimestamp, weightValue, motorPos);

        // Łączymy oba obrazy
        Mat finalImg = concatImageAndInfoBar(thermalImg, tsBar);

        // Wyświetlamy
        imshow("Thermal Image", finalImg);

        int key = waitKey(1);
        if (key == 27) break; // ESC

        if (key >= 'A' && key <= 'Z') {
            char c = static_cast<char>(key);
            char newline = '\n';

            // Wyślij literę
            if (write(fd, &c, 1) != 1) {
                cerr << "Błąd wysyłania '" << c << "' przez serial: " << strerror(errno) << "\n";
            }

            // Wyślij nową linię
            if (write(fd, &newline, 1) != 1) {
                cerr << "Błąd wysyłania '\\n' przez serial: " << strerror(errno) << "\n";
            }

            cout << "Wysłano: " << c << "\\n przez serial\n";
        }
    }

    
    uvc_cleanup(ctx);
    close(fd);
    destroyAllWindows();
    return 0;
}
