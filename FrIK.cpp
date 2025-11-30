// g++ -std=c++17 -o frik FrIK.cpp -lzip `pkg-config --cflags --libs opencv4`

#include <iostream>
#include <zip.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <map>
#include <chrono>
#include <atomic>
#include <fstream>
#include <regex>
#include <algorithm>
#include <filesystem>
#include <cstdlib>
#include <ctime>

#define AREA_HEIGHT_PX 1024
#define AREA_HEIGHT_CM 10

#define DARK_THRESHOLD 120      //LDF: 100
#define BRIGHT_THRESHOLD 240    //LDF: 180
#define BLUE_THRESHOLD 150      //LDF: 150
#define MIN_AREA_PERCENT 1.0  // minimalne pole w procentach obrazu
#define MIN_CIRCULARITY 0.55

bool analyse_only = 0;

// Struktura do przechowywania wczytanych danych PNG
struct PngEntry {
    std::string name;
    std::vector<unsigned char> data;
    int frameNumber; // nowa wartość liczby w nazwie
};

std::vector<PngEntry> pngBuffers;
    
// Używamy namespace fs (C++17)
namespace fs = std::filesystem;

// Globalna lista środków (dla każdego boxa osobna lista)
std::vector<std::vector<cv::Point2f>> g_circleCenters;

int thresholdValue = 128;  // domyślny próg

// Struktura do przechowywania pozycji boxów
struct Box {
    cv::Rect rect;
    int id;
};

bool saveWarpedImage(
    const cv::Mat& warped,
    const std::string& originalName,
    const std::string& directoryPath)
{
    // Katalog wyjściowy
    std::string outputDir = directoryPath + "_output";

    // Utwórz katalog jeśli nie istnieje
    std::filesystem::create_directories(outputDir);

    // Pełna ścieżka do pliku
    std::string outputPath = outputDir + "/" + originalName;

    // Zapis
    return cv::imwrite(outputPath, warped);
}

void whiteOutOutsideBox(cv::Mat &mask, const Box &box)
{
    CV_Assert(mask.type() == CV_8UC1 && "mask must be single-channel 8-bit image");

    // prostokąt w granicach obrazu
    cv::Rect roi = box.rect & cv::Rect(0, 0, mask.cols, mask.rows);

    // 1️⃣ powyżej prostokąta
    if (roi.y > 0)
        mask(cv::Rect(0, 0, mask.cols, roi.y)) = 255;

    // 2️⃣ poniżej prostokąta
    if (roi.y + roi.height < mask.rows)
        mask(cv::Rect(0, roi.y + roi.height, mask.cols, mask.rows - (roi.y + roi.height))) = 255;

    // 3️⃣ na prawo od prostokąta
    if (roi.x + roi.width < mask.cols)
        mask(cv::Rect(roi.x + roi.width, roi.y, mask.cols - (roi.x + roi.width), roi.height)) = 255;
}

void fillMaskFromEdges(cv::Mat &mask)
{
    CV_Assert(mask.type() == CV_8UC1 && "fillMaskFromEdges: mask must be single-channel 8-bit image");

    // Górna i dolna krawędź
    for (int x = 0; x < mask.cols; x++)
    {
        if (mask.at<uchar>(0, x) == 255)
            cv::floodFill(mask, cv::Point(x, 0), 0);
        if (mask.at<uchar>(mask.rows - 1, x) == 255)
            cv::floodFill(mask, cv::Point(x, mask.rows - 1), 0);
    }

    // Lewa i prawa krawędź
    for (int y = 0; y < mask.rows; y++)
    {
        if (mask.at<uchar>(y, 0) == 255)
            cv::floodFill(mask, cv::Point(0, y), 0);
        if (mask.at<uchar>(y, mask.cols - 1) == 255)
            cv::floodFill(mask, cv::Point(mask.cols - 1, y), 0);
    }
}

void imshowScaled(const std::string& winName, const cv::Mat& img, int maxWidth = 800, int maxHeight = 600)
{
    if(analyse_only){
        return;
    }
    
    cv::Mat resized;
    double scale = std::min(
        static_cast<double>(maxWidth) / img.cols,
        static_cast<double>(maxHeight) / img.rows
    );

    if (scale < 1.0)
        cv::resize(img, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    else
        resized = img;

    cv::imshow(winName, resized);
}

// Funkcja do liczenia czarnych pikseli w obrazie fireBlob
int countBlackPixels(const cv::Mat& fireBlob) {
    // Sprawdzamy, czy obraz jest pusty
    if (fireBlob.empty()) {
        std::cerr << "Obraz fireBlob jest pusty!" << std::endl;
        return -1;
    }

    // Sprawdzamy, czy obraz jest w skali szarości (jeden kanał)
    if (fireBlob.channels() == 1) {
        // Tworzymy binarną maskę, gdzie 0 to czarne piksele, a 255 to inne
        cv::Mat blackPixels;
        cv::inRange(fireBlob, cv::Scalar(0), cv::Scalar(0), blackPixels);  // Szukamy pikseli o wartości 0 (czarne)

        // Liczymy liczbę pikseli równych 0 (czarne)
        return cv::countNonZero(blackPixels);
    }
    // Dla obrazu kolorowego (trzy kanały RGB)
    else if (fireBlob.channels() == 3) {
        // Tworzymy maskę, gdzie czarne piksele (wszystkie kanały = 0) będą miały wartość 255
        cv::Mat blackPixels;
        cv::inRange(fireBlob, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), blackPixels);

        // Liczymy liczbę czarnych pikseli
        return cv::countNonZero(blackPixels);
    }
    else {
        std::cerr << "Nieobsługiwany format obrazu!" << std::endl;
        return -1;
    }
}

void keepLargestBlob(cv::Mat& thresh)
{
    CV_Assert(thresh.type() == CV_8UC1);

    int imgArea = thresh.rows * thresh.cols;
    double minArea = (MIN_AREA_PERCENT / 100.0) * imgArea;

    // 🔹 Odwróć obraz, żeby czarne bloby stały się białe
    cv::Mat inv;
    cv::bitwise_not(thresh, inv);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(inv.clone(), contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        thresh.setTo(255); // jeśli brak blobów, zostaw cały obraz biały
        return;
    }

    // 🔹 znajdź największy blob
    double maxArea = 0;
    int maxIdx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea && area > minArea) {
            maxArea = area;
            maxIdx = static_cast<int>(i);
        }
    }
    
    if(maxIdx < 0){
        thresh.setTo(255); // jeśli brak blobów, zostaw cały obraz biały
        return;
    }

    // 🔹 stwórz obraz wyjściowy wypełniony bielą
    cv::Mat mask = cv::Mat::ones(thresh.size(), CV_8UC1) * 255;

    std::cout << imgArea << " > " << maxArea << " > " << minArea << std::endl;
    cv::drawContours(mask, contours, maxIdx, cv::Scalar(0), cv::FILLED);

    //cv::imshow("M", mask);
    
    thresh = mask; // zmiana obrazu przez referencję
}

struct DetectedLine {
    float rho;
    float theta;
    double length;
    cv::Point pt1, pt2;
};

// Struktura do przechowywania linii
struct Line {
    cv::Point start;
    cv::Point end;
    int length;
};

void findLongestLines(const cv::Mat& thresh, Line &hLine, Line &vLine)
{
    CV_Assert(thresh.type() == CV_8UC1);

    int maxHLen = 0;
    cv::Point hStart, hEnd;

    int maxVLen = 0;
    cv::Point vStart, vEnd;

    // Najdłuższa pozioma linia
    for (int y = 0; y < thresh.rows; y++) {
        int currentLen = 0;
        int xStart = 0;

        for (int x = 0; x < thresh.cols; x++) {
            if (thresh.at<uchar>(y, x) == 0) { // czarny piksel
                if (currentLen == 0) xStart = x;
                currentLen++;
                if (currentLen > maxHLen) {
                    maxHLen = currentLen;
                    hStart = {xStart, y};
                    hEnd   = {x, y};
                }
            } else {
                currentLen = 0;
            }
        }
    }

    // Najdłuższa pionowa linia
    for (int x = 0; x < thresh.cols; x++) {
        int currentLen = 0;
        int yStart = 0;

        for (int y = 0; y < thresh.rows; y++) {
            if (thresh.at<uchar>(y, x) == 0) { // czarny piksel
                if (currentLen == 0) yStart = y;
                currentLen++;
                if (currentLen > maxVLen) {
                    maxVLen = currentLen;
                    vStart = {x, yStart};
                    vEnd   = {x, y};
                }
            } else {
                currentLen = 0;
            }
        }
    }

    // Wypełnianie struktur
    hLine = {hStart, hEnd, maxHLen};
    vLine = {vStart, vEnd, maxVLen};
}

// ------------------ findCentralLine ------------------
Line findCentralLine(const cv::Mat& thresh)
{
    CV_Assert(thresh.type() == CV_8UC1);

    int rows = thresh.rows;
    int cols = thresh.cols;

    cv::Point leftMost, rightMost;

    // od lewej do prawej
    for (int x = 0; x < cols; x++) {
        int sumY = 0;
        int count = 0;
        for (int y = 0; y < rows; y++) {
            if (thresh.at<uchar>(y, x) == 0) {
                sumY += y;
                count++;
            }
        }
        if (count > 0) {
            leftMost = {x, sumY / count};
            break;
        }
    }

    // od prawej do lewej
    for (int x = cols - 1; x >= 0; x--) {
        int sumY = 0;
        int count = 0;
        for (int y = 0; y < rows; y++) {
            if (thresh.at<uchar>(y, x) == 0) {
                sumY += y;
                count++;
            }
        }
        if (count > 0) {
            rightMost = {x, sumY / count};
            break;
        }
    }

    int length = cv::norm(rightMost - leftMost);
    return {leftMost, rightMost, length};
}

// ------------------ Wyświetlanie linii ------------------
void drawLines(cv::Mat& color, const Line &hLine, const Line &vLine, const Line &cLine)
{
    //cv::Mat color;
    //cv::cvtColor(thresh, color, cv::COLOR_GRAY2BGR);

    if (hLine.length > 0)
        cv::line(color, hLine.start, hLine.end, cv::Scalar(0,255,0), 2); // zielona pozioma
    if (vLine.length > 0)
        cv::line(color, vLine.start, vLine.end, cv::Scalar(255,0,0), 2); // niebieska pionowa
    if (cLine.length > 0)
        cv::line(color, cLine.start, cLine.end, cv::Scalar(0,255,255), 2); // cyjan środkowa

    //imshowScaled("All Lines", color);
}

// ==========================
// 🔹 Pomocnicze funkcje
// ==========================

// Tworzy maskę bardzo ciemnych pikseli
void createDarkMask(const cv::Mat& blurred, cv::Mat& outMask, int thr) {
    //std::cout << "Darl: " << thr << std::endl;
    cv::threshold(blurred, outMask, thr, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_not(outMask, outMask);  // interesujące = czarne
}

// Tworzy maskę bardzo jasnych pikseli
void createBrightMask(const cv::Mat& blurred, cv::Mat& outMask, int thr) {
    cv::threshold(blurred, outMask, thr, 255, cv::THRESH_BINARY);
    cv::bitwise_not(outMask, outMask);  // interesujące = czarne
}

// Przykład dodatkowej maski (np. niebieska barwa z oryginału)
void createBlueMask(const cv::Mat& warped, cv::Mat& outMask, int thresholdVal = BLUE_THRESHOLD) {
    std::vector<cv::Mat> channels;
    cv::split(warped, channels); // [B, G, R]
    cv::threshold(channels[0], outMask, thresholdVal, 255, cv::THRESH_BINARY_INV);
}

// Erozja – redukuje małe plamy
cv::Mat erodeMask(const cv::Mat& mask, int size = 5) {
    cv::Mat result;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * size + 1, 2 * size + 1),
        cv::Point(size, size)
    );
    cv::erode(mask, result, element);
    return result;
}

// Dylatacja – przywraca obszary po erozji
cv::Mat dilateMask(const cv::Mat& mask, int size = 5) {
    cv::Mat result;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * size + 1, 2 * size + 1),
        cv::Point(size, size)
    );
    cv::dilate(mask, result, element);
    return result;
}

// ==========================
// 🔹 Pomocnicza funkcja flood fill (wypełnianie dziur w czarnej masce)
// ==========================
void fillHoles(cv::Mat& mask) {
    if (mask.empty()) return;

    cv::Mat flood = mask.clone();
    
    // Flood fill wypełnia tło od punktu (0,0)
    cv::floodFill(flood, cv::Point(0, 0), 128);

    // Zamień oryginalne tło (255) na 0 i flood na 255
    mask.setTo(0,   mask == 255);
    mask.setTo(255, flood == 128);
}

// ==========================
// 🔹 Główna funkcja
// ==========================
cv::Mat highlightBurnEdges(cv::Mat& warped, double thresholdRatio,
                           const Box* fifthBox, 
                           int selectedBox,
                           std::vector<std::vector<int>>& params) {
    if (warped.empty()) {
        std::cerr << "[Błąd] Pusty obraz w highlightBurnEdges()." << std::endl;
        return cv::Mat();
    }

    // 1️⃣ Szarość i blur
    cv::Mat gray, blurred;
    cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);

    // 2️⃣ Maski składowe
    cv::Mat darkMask, brightMask, blueMask;
    createDarkMask(blurred, darkMask, params[0][0]);
    createBrightMask(blurred, brightMask, params[1][0]);
    createBlueMask(warped, blueMask, params[2][0]);
    
    imshowScaled("Dark Mask (keys w/e)", darkMask);
    imshowScaled("Bright Mask (keys s/d)", brightMask);
    imshowScaled("Blue Mask (keys x/c)", blueMask);
    
    whiteOutOutsideBox(darkMask, *fifthBox);
    whiteOutOutsideBox(brightMask, *fifthBox);
    whiteOutOutsideBox(blueMask, *fifthBox);
    
    fillHoles(darkMask);
    fillHoles(brightMask);
    fillHoles(blueMask);
    
    darkMask = erodeMask(darkMask);
    brightMask = erodeMask(brightMask);
    blueMask = erodeMask(blueMask);

    // 3️⃣ Połącz maski – czarne pola są interesujące
    cv::Mat combined;
    cv::bitwise_and(darkMask, brightMask, combined);
    cv::bitwise_and(combined, blueMask, combined);

    fillHoles(combined);
    combined = dilateMask(combined);
    combined = dilateMask(combined);
    //fillHoles(combined);
    
    //imshow("c", combined);
    
    // 4️⃣ Redukcja plam (erozja + dylatacja)
    cv::Mat shrunken = erodeMask(combined, 5);
    cv::Mat thresh   = dilateMask(shrunken, 5);

    // 5️⃣ Czyszczenie 
    keepLargestBlob(thresh);

    // 6️⃣ Krawędzie (Canny)
    cv::Mat edges;
    cv::Canny(thresh, edges, 50, 150);
    
    //imshow("e", edges);

    // 7️⃣ Nałóż krawędzie na oryginał
    cv::Mat result = warped.clone();
    for (int y = 0; y < edges.rows; ++y) {
        for (int x = 0; x < edges.cols; ++x) {
            if (edges.at<uchar>(y, x) > 0)
                result.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
        }
    }

    // 8️⃣ Rysowanie piątego boxa
    if (fifthBox != nullptr) {
        cv::Scalar color = (selectedBox == 5) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0);
        cv::rectangle(result, fifthBox->rect, color, 2);
        cv::putText(result, std::to_string(fifthBox->id),
                    cv::Point(fifthBox->rect.x + fifthBox->rect.width + 5, fifthBox->rect.y + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    warped = result;
    return thresh;
}



/*void saveBoxPositionsToFile(const std::string& zipFileName, const std::vector<Box>& boxes, int threshold) {
    std::string fileName = zipFileName.substr(0, zipFileName.find_last_of('.')) + ".box";
    std::ofstream outFile(fileName);

    if (!outFile.is_open()) {
        std::cerr << "Błąd przy otwieraniu pliku do zapisu: " << fileName << std::endl;
        return;
    }

    // Zapisz threshold w pierwszej linii
    outFile << "threshold " << threshold << std::endl;

    // Zapisz pozycje i rozmiary boxów
    for (const auto& box : boxes) {
        outFile << box.id << " "
                << box.rect.x << " " << box.rect.y << " "
                << box.rect.width << " " << box.rect.height << std::endl;
    }

    outFile.close();
    std::cout << "Zapisano pozycje boxów i threshold do pliku: " << fileName << std::endl;
}*/

void saveBoxPositionsToFile(const std::string& zipFileName, const std::vector<Box>& boxes, const std::vector<std::vector<int>>& masks, int threshold) {
    std::string fileName = zipFileName + ".box";
    std::ofstream outFile(fileName);

    if (!outFile.is_open()) {
        std::cerr << "Błąd przy otwieraniu pliku do zapisu: " << fileName << std::endl;
        return;
    }

    // Zapisz threshold w pierwszej linii
    outFile << "threshold " << threshold << std::endl;

    // Zapisz pozycje, rozmiary boxów
    for (const auto& box : boxes) {
        outFile << box.id << " "
                << box.rect.x << " " << box.rect.y << " "
                << box.rect.width << " " << box.rect.height << std::endl;
    }

    // Zapisz maski jako osobny blok
    outFile << "masks ";
    for (const auto& mask : masks) {
        // Zapisz każdy element wektora maski (w tym przypadku tylko jeden)
        for (int value : mask) {
            outFile << value << " ";  // Zapisz wartość
        }
    }
    outFile << std::endl;

    outFile.close();
    std::cout << "Zapisano pozycje boxów i threshold do pliku: " << fileName << std::endl;
}

// Funkcja, która generuje nagłówek
std::string generateHeader() {
    // Nagłówek, zawierający odpowiednie etykiety i jednostki
    std::stringstream header;
    header << "name frameNumber hLine(px) vLine(px) cLine(px) blackPixelCount(px^2) "
           << "hLine(cm) vLine(cm) cLine(cm) blackPixelCount(cm^2)";
    return header.str();
}

// Funkcja, która przelicza długości na centymetry i powierzchnię na cm^2
std::string processResults(const Line& hLine, const Line& vLine, const Line& cLine,
                             int blackPixelCount, const std::vector<PngEntry>& pngBuffers, int currentImageIndex) {

    // Przeliczenie długości na centymetry
    float hLineLengthCM = static_cast<float>(hLine.length) * AREA_HEIGHT_CM / AREA_HEIGHT_PX;
    float vLineLengthCM = static_cast<float>(vLine.length) * AREA_HEIGHT_CM / AREA_HEIGHT_PX;
    float cLineLengthCM = static_cast<float>(cLine.length) * AREA_HEIGHT_CM / AREA_HEIGHT_PX;

    // Przeliczenie powierzchni na cm^2
    float blackPixelCountAreaCM2 = static_cast<float>(blackPixelCount) * (AREA_HEIGHT_CM * AREA_HEIGHT_CM) / (AREA_HEIGHT_PX * AREA_HEIGHT_PX);

    // Pobranie danych z pngBuffers
    const PngEntry& entry = pngBuffers[currentImageIndex];

    // Tworzenie wiersza
    std::stringstream result;
    result << entry.name << " "
           << entry.frameNumber << " "
           << hLine.length << " " << vLine.length << " " << cLine.length << " " << blackPixelCount << " "  // Wartości w pikselach
           << hLineLengthCM << " " << vLineLengthCM << " " << cLineLengthCM << " " << blackPixelCountAreaCM2;  // Wartości w centymetrach

    return result.str();
}

/*bool loadBoxPositionsFromFile(const std::string& zipFileName, std::vector<Box>& boxes, int& threshold) {
    std::string fileName = zipFileName.substr(0, zipFileName.find_last_of('.')) + ".box";
    std::ifstream inFile(fileName);

    if (!inFile.is_open()) {
        std::cerr << "Plik " << fileName << " nie istnieje, wczytuję z default.box." << std::endl;
        inFile.open("default.box");
    }

    if (!inFile.is_open()) {
        std::cerr << "Błąd przy otwieraniu pliku do odczytu!" << std::endl;
        return false;
    }

    boxes.clear();
    threshold = 128; // domyślna wartość jeśli nie znajdziemy w pliku

    std::string line;
    bool thresholdRead = false;

    while (std::getline(inFile, line)) {
        std::istringstream iss(line);
        std::string firstWord;
        iss >> firstWord;

        if (!thresholdRead && firstWord == "threshold") {
            iss >> threshold;
            thresholdRead = true;
        } else {
            // Wczytaj boxy
            int id, x, y, w, h;
            try {
                id = std::stoi(firstWord);
                iss >> x >> y >> w >> h;
                boxes.push_back({cv::Rect(x, y, w, h), id});
            } catch (...) {
                std::cerr << "Błąd formatu w linii: " << line << std::endl;
            }
        }
    }

    inFile.close();
    return true;
}*/

bool loadBoxPositionsFromFile(const std::string& zipFileName, std::vector<Box>& boxes, std::vector<std::vector<int>>& masks, int& threshold) {
    std::string fileName = zipFileName + ".box";
    std::ifstream inFile(fileName);

    if (!inFile.is_open()) {
        std::cerr << "Plik " << fileName << " nie istnieje, wczytuję z default.box." << std::endl;
        inFile.open("default.box");
    }

    if (!inFile.is_open()) {
        std::cerr << "Błąd przy otwieraniu pliku do odczytu!" << std::endl;
        return false;
    }

    boxes.clear();
    masks.clear();
    threshold = 128; // domyślna wartość jeśli nie znajdziemy w pliku

    std::string line;
    bool thresholdRead = false;

    while (std::getline(inFile, line)) {
        std::istringstream iss(line);
        std::string firstWord;
        iss >> firstWord;

        if (!thresholdRead && firstWord == "threshold") {
            iss >> threshold;
            thresholdRead = true;
            std::cout << "Wczytano threshold: " << threshold << std::endl; // Wyświetlanie threshold
        }else if (firstWord == "masks") {
            // Wczytanie masek po "masks"
            std::vector<int> mask;
            int value;

            // Wczytujemy wartości z linii i przypisujemy je do pojedynczych masek
            while (iss >> value) {
                // Każdą liczbę traktujemy jako osobną maskę
                std::vector<int> singleMask = {value};  // Tworzymy wektor z jednym elementem
                masks.push_back(singleMask);  // Dodajemy do głównej listy masek

                std::cout << "Wczytano maskę: " << value << std::endl;
            }
        }
         else {
            // Wczytaj boxy
            int id, x, y, w, h;
            try {
                id = std::stoi(firstWord);
                iss >> x >> y >> w >> h;
                boxes.push_back({cv::Rect(x, y, w, h), id});

                std::cout << "Wczytano box: ID = " << id << ", "
                          << "Pozycja = (" << x << ", " << y << "), "
                          << "Rozmiar = (" << w << "x" << h << ")" << std::endl; // Wyświetlanie boxów

            } catch (...) {
                std::cerr << "Błąd formatu w linii: " << line << std::endl;
            }
        }
    }

    inFile.close();
    return true;
}




/*
cv::Mat generateImage(const cv::Mat& image, const std::vector<Box>& boxes, int selectedBox) {
    // Skalowanie obrazu do max 800x600 z zachowaniem proporcji
    int maxWidth = 800;
    int maxHeight = 600;
    int imgWidth = image.cols;
    int imgHeight = image.rows;

    double scale = std::min((double)maxWidth / imgWidth, (double)maxHeight / imgHeight);
    int newWidth = static_cast<int>(imgWidth * scale);
    int newHeight = static_cast<int>(imgHeight * scale);

    // Skalujemy obraz
    cv::Mat resizedImage;
    cv::resize(image, resizedImage, cv::Size(newWidth, newHeight));

    // Rysowanie przeskalowanych boxów
    for (const auto& box : boxes) {
        // Skalowanie pozycji i rozmiaru boxa
        cv::Rect scaledRect;
        scaledRect.x = static_cast<int>(box.rect.x * scale);
        scaledRect.y = static_cast<int>(box.rect.y * scale);
        scaledRect.width = static_cast<int>(box.rect.width * scale);
        scaledRect.height = static_cast<int>(box.rect.height * scale);

        // Wybór koloru
        cv::Scalar color = (box.id == selectedBox) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

        // Rysuj box
        cv::rectangle(resizedImage, scaledRect, color, 2);

        // Opcjonalnie: narysuj numer boxa przy nim
        cv::putText(resizedImage, std::to_string(box.id),
                    cv::Point(scaledRect.x + scaledRect.width + 5, scaledRect.y + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    return resizedImage;
}
*/

cv::Mat generateImage(const cv::Mat& image, const std::vector<Box>& boxes, int selectedBox) {
    // Skalowanie obrazu do max 800x600 z zachowaniem proporcji
    int maxWidth = 800;
    int maxHeight = 600;
    int imgWidth = image.cols;
    int imgHeight = image.rows;

    double scale = std::min((double)maxWidth / imgWidth, (double)maxHeight / imgHeight);
    int newWidth = static_cast<int>(imgWidth * scale);
    int newHeight = static_cast<int>(imgHeight * scale);

    // Skalujemy obraz
    cv::Mat resizedImage;
    cv::resize(image, resizedImage, cv::Size(newWidth, newHeight));
    
    //std::cout << "debug1" << std::endl;

    // Rysowanie przeskalowanych boxów (tylko pierwsze 4)
    int count = 0;
    for (const auto& box : boxes) {
        if (count >= 4) break;  // malujemy tylko pierwsze 4 boxy
        count++;

        // Skalowanie pozycji i rozmiaru boxa
        cv::Rect scaledRect;
        scaledRect.x = static_cast<int>(box.rect.x * scale);
        scaledRect.y = static_cast<int>(box.rect.y * scale);
        scaledRect.width = static_cast<int>(box.rect.width * scale);
        scaledRect.height = static_cast<int>(box.rect.height * scale);

        // Wybór koloru
        cv::Scalar color;
        if (box.id == selectedBox) {
            color = cv::Scalar(0, 255, 0); // zielony dla wybranego boxa
        //} else if (box.id == 5) {
        //    color = cv::Scalar(255, 0, 0); // niebieski dla piątego boxa (nie będzie narysowany)
        } else {
            color = cv::Scalar(0, 0, 255); // czerwony dla pozostałych
        }

        // Rysuj box
        cv::rectangle(resizedImage, scaledRect, color, 2);

        // Narysuj numer boxa przy nim
        cv::putText(resizedImage, std::to_string(box.id),
                    cv::Point(scaledRect.x + scaledRect.width + 5, scaledRect.y + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    //std::cout << "debug2" << std::endl;
    
    return resizedImage;
}


// Funkcja do wyświetlania obrazu
void displayImage(const cv::Mat& image) {
    cv::imshow("Viewer (tab, arrows, j/k/i/l)", image);
}

// Funkcja do obsługi klawiszy
void handleKeys(char key, int& currentImageIndex, int& selectedBox, std::vector<Box>& boxes, int& thresholdValue, std::vector<std::vector<int>>& masks) {
    if (key == 27 || key == 'q') {  // Esc
        std::cout << "Zamykanie programu..." << std::endl;
        std::exit(0);

    } else if (key == 32) {  // Spacja
        currentImageIndex++;

    } else if (key == 8) {  // Backspace
        currentImageIndex = (currentImageIndex == 0) ? 0 : currentImageIndex - 1;

    } else if (key == 81 || key == 'j') {  // ←
        boxes[selectedBox - 1].rect.x -= 10;

    } else if (key == 83 || key == 'l') {  // →
        boxes[selectedBox - 1].rect.x += 10;

    } else if (key == 82 || key == 'i') {  // ↑
        boxes[selectedBox - 1].rect.y -= 10;

    } else if (key == 84 || key == 'k') {  // ↓
        boxes[selectedBox - 1].rect.y += 10;

    } else if (key == 9) {  // Tab
        selectedBox = (selectedBox % boxes.size()) + 1;
        std::cout << "Wybrano Box " << selectedBox << std::endl;

    } else if (key == 353) {  // Shift+Tab
        selectedBox = (selectedBox == 1) ? boxes.size() : selectedBox - 1;
        std::cout << "Wybrano Box " << selectedBox << std::endl;

    } else if (key == '+' || key == 61) {
        boxes[selectedBox - 1].rect.height += 5;
        if(selectedBox < 5){
            //boxes[selectedBox - 1].rect.height = 2 * boxes[selectedBox - 1].rect.width;
            boxes[selectedBox - 1].rect.width = boxes[selectedBox - 1].rect.height;
        }

    } else if (key == '-' || key == 45) {
        boxes[selectedBox - 1].rect.height = std::max(5, boxes[selectedBox - 1].rect.height - 5);
        if(selectedBox < 5){
            //boxes[selectedBox - 1].rect.height = 2 * boxes[selectedBox - 1].rect.width;
            boxes[selectedBox - 1].rect.width = boxes[selectedBox - 1].rect.height;
        }

    } else if (key == 't') {  // zmniejsz threshold
        thresholdValue = std::max(0, thresholdValue - 5);
        std::cout << "Threshold zmniejszony: " << thresholdValue << std::endl;

    } else if (key == 'y') {  // zwiększ threshold
        thresholdValue = std::min(255, thresholdValue + 5);
        std::cout << "Threshold zwiększony: " << thresholdValue << std::endl;
    } else if (key == 'r' || key == 'R') {  // losowy obraz
        currentImageIndex = std::rand() % pngBuffers.size();
        std::cout << "🔀 Wybrano losowy obraz: " << pngBuffers[currentImageIndex].name
                << " (index " << currentImageIndex << ")" << std::endl;

    // Zmiana wartości masek (DARK_THRESHOLD, BRIGHT_THRESHOLD, BLUE_THRESHOLD)
    } else if (key == 'w') {  // Zmniejsz pierwszą maskę (DARK_THRESHOLD)
        masks[0][0] = std::max(0, masks[0][0] - 1);
        std::cout << "Zmniejszono DARK_THRESHOLD na: " << masks[0][0] << std::endl;
    } else if (key == 'e') {  // Zwiększ pierwszą maskę (DARK_THRESHOLD)
        masks[0][0] = std::min(255, masks[0][0] + 1);
        std::cout << "Zwiększono DARK_THRESHOLD na: " << masks[0][0] << std::endl;
    
    } else if (key == 's') {  // Zmniejsz drugą maskę (BRIGHT_THRESHOLD)
        masks[1][0] = std::max(0, masks[1][0] - 1);
        std::cout << "Zmniejszono BRIGHT_THRESHOLD na: " << masks[1][0] << std::endl;
    } else if (key == 'd') {  // Zwiększ drugą maskę (BRIGHT_THRESHOLD)
        std::cout << "1: " << masks[1][0] << std::endl;
        masks[1][0] = std::min(255, masks[1][0] + 1);
        std::cout << "Zwiększono BRIGHT_THRESHOLD na: " << masks[1][0] << std::endl;
        std::cout << "1: " << masks[1][0] << std::endl;
    
    } else if (key == 'x') {  // Zmniejsz trzecią maskę (BLUE_THRESHOLD)
        masks[2][0] = std::max(0, masks[2][0] - 1);
        std::cout << "Zmniejszono BLUE_THRESHOLD na: " << masks[2][0] << std::endl;
    } else if (key == 'c') {  // Zwiększ trzecią maskę (BLUE_THRESHOLD)
        masks[2][0] = std::min(255, masks[2][0] + 1);
        std::cout << "Zwiększono BLUE_THRESHOLD na: " << masks[2][0] << std::endl;
    }
}



// Funkcja do wyświetlania aktualnych pozycji wszystkich boxów
void displayBoxPositions(const std::vector<Box>& boxes) {
    for (const auto& box : boxes) {
        std::cout << "Box " << box.id << " - Pozycja: ("
                  << box.rect.x << ", " << box.rect.y << "), "
                  << "Rozmiar: (" << box.rect.width << "x" << box.rect.height << ")" << std::endl;
    }
}

void displayBoxPositionsEvery5s(const std::vector<Box>& boxes) {
    static auto lastDisplay = std::chrono::steady_clock::now();

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - lastDisplay;

    if (elapsed.count() >= 5.0) {
        std::cout << "\n--- Pozycje boxów (co 5s) ---\n";
        displayBoxPositions(boxes);
        lastDisplay = now;
    }
}

void detectCirclesInBoxes(const cv::Mat& originalImage, const std::vector<Box>& boxes, int thresholdValue) {
    g_circleCenters.clear();
    g_circleCenters.resize(4);

    for (size_t i = 0; i < 4; ++i) {
        const auto& box = boxes[i];

        cv::Rect clippedBox = box.rect & cv::Rect(0, 0, originalImage.cols, originalImage.rows);
        if (clippedBox.area() <= 0) continue;

        cv::Mat roi = originalImage(clippedBox);
        cv::Mat resized;
        cv::resize(roi, resized, cv::Size(300, 300));

        cv::Mat gray;
        cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

        cv::Mat stretched;
        cv::normalize(gray, stretched, 0, 255, cv::NORM_MINMAX);

        cv::Mat binaryImage;
        cv::threshold(stretched, binaryImage, thresholdValue, 255, cv::THRESH_BINARY);
        
        
        binaryImage = erodeMask(binaryImage);
        fillMaskFromEdges(binaryImage);
        binaryImage = erodeMask(binaryImage);
       
//         if(i == 1){
//             imshow("bin", binaryImage);
//         }
        
        int s = 75;
        
        cv::Mat binaryClosed;
        cv::morphologyEx(binaryImage, binaryClosed, cv::MORPH_CLOSE,
                        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(s, s)));
                
        cv::dilate(binaryClosed, binaryClosed,
           cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(s, s)));
                
        // Znajdź kontury
        std::vector<std::vector<cv::Point>> contours;
        
        
        cv::findContours(binaryClosed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        //cv::findContours(binaryImage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 100.0) {
                std::cout << "Box " << i << ": Rejecting too small." << std::endl;
                continue;
            }

            double perimeter = cv::arcLength(contour, true);
            if (perimeter <= 0) {
                std::cout << "Box " << i << ": Rejecting too small perimeter." << std::endl;
                continue;
            }

            double circularity = 4 * CV_PI * area / (perimeter * perimeter);
            if (circularity < MIN_CIRCULARITY) {
                std::cout << "Box " << i << ": Rejecting too small circularity: " << circularity << std::endl;
                continue;
            }
            
            if(circularity > 1.2) {
                std::cout << "Box " << i << ": Rejecting too big circularity." << std::endl;
                continue;
            }

            cv::Moments m = cv::moments(contour);
            if (m.m00 == 0) {
                std::cout << "Box " << i << ": Rejecting moments=0." << std::endl;
                continue;
            }

            cv::Point2f center(static_cast<float>(m.m10 / m.m00),
                               static_cast<float>(m.m01 / m.m00));

            g_circleCenters[i].push_back(center);
        }
    }
}

void showDetectedCircles(const cv::Mat& originalImage, const std::vector<Box>& boxes, int thresholdValue) {
    cv::Mat display(600, 600, CV_8UC3, cv::Scalar(30, 30, 30));

    for (size_t i = 0; i < 4; ++i) {
        const auto& box = boxes[i];

        cv::Rect clippedBox = box.rect & cv::Rect(0, 0, originalImage.cols, originalImage.rows);
        if (clippedBox.area() <= 0) continue;

        cv::Mat roi = originalImage(clippedBox);
        cv::Mat resized;
        cv::resize(roi, resized, cv::Size(300, 300));

        cv::Mat gray;
        cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

        cv::Mat stretched;
        cv::normalize(gray, stretched, 0, 255, cv::NORM_MINMAX);

        cv::Mat binaryImage;
        cv::threshold(stretched, binaryImage, thresholdValue, 255, cv::THRESH_BINARY);

        cv::Mat colorDisplay;
        cv::cvtColor(binaryImage, colorDisplay, cv::COLOR_GRAY2BGR);

        // Rysuj środki z globalnej listy
        for (const auto& center : g_circleCenters[i]) {
            cv::drawMarker(colorDisplay, center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
            cv::circle(colorDisplay, center, 5, cv::Scalar(0, 255, 0), 2);
        }

        int index = box.id - 1;
        int row = index / 2;
        int col = index % 2;
        colorDisplay.copyTo(display(cv::Rect(col * 300, row * 300, 300, 300)));
    }

    std::string text = "Threshold: " + std::to_string(thresholdValue);
    cv::putText(display, text, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

    cv::imshow("Holes binarisation (keys (t/y)", display);
}

// Wczytuje wszystkie poprawne PNG z archiwum ZIP do pamięci
/*bool loadZipAndListPNGs(const std::string& zipFilePath, zip*& archive, std::vector<PngEntry>& pngBuffers) {
    int err = 0;
    archive = zip_open(zipFilePath.c_str(), 0, &err);
    if (archive == nullptr) {
        std::cerr << "Błąd przy otwieraniu pliku ZIP!" << std::endl;
        return false;
    }

    zip_int64_t numFiles = zip_get_num_entries(archive, 0);
    std::regex frameRegex(R"(frame_(\d+)\.png)", std::regex_constants::icase);

    for (zip_int64_t i = 0; i < numFiles; ++i) {
        const char* fileName = zip_get_name(archive, i, 0);
        if (!fileName) continue;

        std::string name(fileName);
        std::smatch match;
        if (!std::regex_match(name, match, frameRegex)) continue;

        int frameNum = std::stoi(match[1].str());

        zip_stat_t stat;
        if (zip_stat_index(archive, i, 0, &stat) != 0) {
            std::cerr << "Nie udało się pobrać statystyk dla " << name << std::endl;
            continue;
        }

        zip_file* zf = zip_fopen_index(archive, i, 0);
        if (!zf) {
            std::cerr << "Nie udało się otworzyć pliku " << name << " z archiwum." << std::endl;
            continue;
        }

        std::vector<unsigned char> buffer(stat.size);
        zip_int64_t bytesRead = zip_fread(zf, buffer.data(), stat.size);
        zip_fclose(zf);

        if (bytesRead != static_cast<zip_int64_t>(stat.size)) {
            std::cerr << "Nie udało się wczytać pełnych danych pliku " << name << std::endl;
            continue;
        }

        cv::Mat testImg = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
        if (testImg.empty()) {
            std::cerr << "Plik " << name << " nie jest prawidłowym PNG lub jest uszkodzony." << std::endl;
            continue;
        }

        pngBuffers.push_back({name, std::move(buffer), frameNum});
    }

    zip_close(archive);
    archive = nullptr;

    if (pngBuffers.empty()) {
        std::cerr << "Brak poprawnych plików PNG w archiwum ZIP!" << std::endl;
        return false;
    }

    // Sortowanie po numerze klatki
    std::sort(pngBuffers.begin(), pngBuffers.end(), [](const PngEntry& a, const PngEntry& b) {
        return a.frameNumber < b.frameNumber;
    });

    return true;
}*/

// ============================
// ZAMIENNIK loadZipAndListPNGs
// ============================
bool loadDirectoryPNGs(const std::string& dirPath, std::vector<PngEntry>& pngBuffers) {
    std::regex frameRegex(R"(frame_(\d+)\.png)", std::regex_constants::icase);

    if (!fs::exists(dirPath) || !fs::is_directory(dirPath)) {
        std::cerr << "Ścieżka " << dirPath << " nie jest katalogiem!" << std::endl;
        return false;
    }

    pngBuffers.clear();

    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (!entry.is_regular_file()) continue;

        std::string fileName = entry.path().filename().string();
        std::smatch match;
        if (!std::regex_match(fileName, match, frameRegex)) continue;

        int frameNum = std::stoi(match[1].str());
        std::ifstream file(entry.path(), std::ios::binary);
        if (!file) continue;

        std::vector<unsigned char> buffer((std::istreambuf_iterator<char>(file)),
                                          std::istreambuf_iterator<char>());

        cv::Mat test = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
        if (test.empty()) {
            std::cerr << "Plik " << fileName << " nie jest poprawnym PNG, pomijam.\n";
            continue;
        }

        pngBuffers.push_back({fileName, std::move(buffer), frameNum});
    }

    if (pngBuffers.empty()) {
        std::cerr << "Brak plików frame_*.png w katalogu!\n";
        return false;
    }

    std::sort(pngBuffers.begin(), pngBuffers.end(),
              [](const PngEntry& a, const PngEntry& b) {
                  return a.frameNumber < b.frameNumber;
              });

    std::cout << "Wczytano " << pngBuffers.size() << " plików PNG z katalogu " << dirPath << "\n";
    return true;
}

// Wczytuje obraz PNG z pamięci bufora
cv::Mat loadPngFromBuffer(const std::vector<PngEntry>& pngBuffers, int& currentImageIndex) {
    if (pngBuffers.empty()) return cv::Mat();

    if (currentImageIndex < 0) currentImageIndex = 0;
    if (currentImageIndex >= static_cast<int>(pngBuffers.size()))
        currentImageIndex = static_cast<int>(pngBuffers.size()) - 1;

    const PngEntry& entry = pngBuffers[currentImageIndex];
    cv::Mat image = cv::imdecode(entry.data, cv::IMREAD_COLOR);

    if (image.empty()) {
        std::cerr << "Błąd przy ładowaniu obrazu PNG z bufora: " << entry.name << std::endl;
    }

    return image;
}

std::vector<Box> createBoxes(int imgWidth, int imgHeight, const std::string& zipFilePath, std::vector<std::vector<int>>& masks) {
    std::vector<Box> boxes = {
        {cv::Rect(imgWidth / 3 - 20, imgHeight / 3 - 20, 40, 40), 1},
        {cv::Rect(imgWidth * 2 / 3 - 20, imgHeight / 3 - 20, 40, 40), 2},
        {cv::Rect(imgWidth / 3 - 20, imgHeight * 2 / 3 - 20, 40, 40), 3},
        {cv::Rect(imgWidth * 2 / 3 - 20, imgHeight * 2 / 3 - 20, 40, 40), 4},
        {cv::Rect(imgWidth / 2 - 20, imgHeight / 2 - 20, 40, 80), 5}
    };

    if (!loadBoxPositionsFromFile(zipFilePath, boxes, masks, thresholdValue)) {
        std::cerr << "Wczytanie danych z pliku nie powiodło się, używam domyślnych boxów!" << std::endl;
    }

    return boxes;
}

std::vector<cv::Point2f> computeCornerPoints(const std::vector<Box>& boxes) {
    std::vector<cv::Point2f> corners;

    // Sprawdź, czy liczba punktów odpowiada liczbie boxów
    if (g_circleCenters.size() != 4) {
        std::cout << "[Błąd] Liczba punktów (" << g_circleCenters.size()
                  << ") nie zgadza się z liczbą boxów (4)."
                  << std::endl;
        return corners;
    }

//     if (boxes.size() < 4) {
//         std::cout << "[Błąd] computeCornerPoints oczekuje dokładnie 4 boxów, otrzymano: "
//                   << boxes.size() << std::endl;
//         return corners;
//     }

    for (size_t i = 0; i < 4; ++i) {
        if (g_circleCenters[i].size() != 1) {
            std::cout << "[Błąd] Box ID=" << boxes[i].id
                      << " ma " << g_circleCenters[i].size()
                      << " punktów (oczekiwano 1)." << std::endl;
            return {};
        }
    }

    // Przekształć współrzędne z przestrzeni boxa do przestrzeni oryginalnego obrazu
    for (size_t i = 0; i < 4; ++i) {
        const auto& box = boxes[i];
        cv::Point2f pt = g_circleCenters[i][0];

        // Przeskalowanie z 300x300 (tam był resize w detekcji) do rzeczywistego boxa
        float scaleX = static_cast<float>(box.rect.width) / 300.0f;
        float scaleY = static_cast<float>(box.rect.height) / 300.0f;

        cv::Point2f correctedPoint(
            box.rect.x + pt.x * scaleX,
            box.rect.y + pt.y * scaleY
        );

        corners.push_back(correctedPoint);
    }

    // Teraz po sortowaniu: [góra-lewa, góra-prawa, dół-lewa, dół-prawa]
    // Zamień miejscami 3. i 4., żeby mieć [TL, TR, BR, BL]
    if (corners.size() == 4)
        std::swap(corners[2], corners[3]);

    return corners;
}

cv::Mat warpImageByCorners(const cv::Mat& originalImage,
                           const std::vector<cv::Point2f>& corners,
                           int outputHeight = 1024,
                           float expandFactor = 0.25f) {
    if (corners.size() < 4) {
        std::cerr << "[Błąd] Oczekiwano dokładnie 4 punkty rogów, otrzymano: "
                  << corners.size() << std::endl;
        return cv::Mat();
    }

    cv::Point2f P0 = corners[0]; // lewy górny
    cv::Point2f P1 = corners[1]; // prawy górny
    cv::Point2f P2 = corners[2]; // prawy dolny
    cv::Point2f P3 = corners[3]; // lewy dolny

    // Pomocnicza funkcja – przesuwa punkt B dalej wzdłuż A->B
    auto extendPoint = [](cv::Point2f A, cv::Point2f B, float factor) {
        cv::Point2f vec = B - A;
        return B + vec * factor;
    };

    // Pomocnicza funkcja – przesuwa punkt A w przeciwną stronę od B
    auto extendOpposite = [](cv::Point2f A, cv::Point2f B, float factor) {
        cv::Point2f vec = A - B;
        return A + vec * factor;
    };

    // 🔹 Przesuń lewe punkty w lewo
    cv::Point2f newP0 = extendOpposite(P0, P1, 2*expandFactor);
    cv::Point2f newP3 = extendOpposite(P3, P2, 2*expandFactor);

    // 🔹 Przesuń prawe punkty w prawo
    cv::Point2f newP1 = extendPoint(P0, P1, expandFactor);
    cv::Point2f newP2 = extendPoint(P3, P2, expandFactor);

    // Oblicz wysokość i szerokość
    double leftHeight = cv::norm(newP3 - newP0);
    double rightHeight = cv::norm(newP2 - newP1);
    double avgHeight = (leftHeight + rightHeight) / 2.0;

    double topWidth = cv::norm(newP1 - newP0);
    double bottomWidth = cv::norm(newP2 - newP3);
    double avgWidth = (topWidth + bottomWidth) / 2.0;

    int outputWidth = static_cast<int>(outputHeight * (avgWidth / avgHeight));

    std::vector<cv::Point2f> dstCorners = {
        cv::Point2f(0, 0),
        cv::Point2f(outputWidth - 1, 0),
        cv::Point2f(outputWidth - 1, outputHeight - 1),
        cv::Point2f(0, outputHeight - 1)
    };

    // 🔹 Nowy rozszerzony czworokąt
    std::vector<cv::Point2f> srcCorners = {newP0, newP1, newP2, newP3};

    cv::Mat transform = cv::getPerspectiveTransform(srcCorners, dstCorners);

    cv::Mat warped;
    cv::warpPerspective(originalImage, warped, transform, cv::Size(outputWidth, outputHeight));

    return warped;
}



cv::Mat warpAndCropFromLeft(const cv::Mat& originalImage,
                            const std::vector<cv::Point2f>& corners,
                            int outputHeight = 1024,
                            int outputWidth = 1280) {
    if (corners.size() < 4) {
        std::cout << "[Błąd] Oczekiwano dokładnie 4 punkty rogów, otrzymano: "
                  << corners.size() << std::endl;
        return cv::Mat();
    }

    // Obliczamy wysokość prostokąta (outputHeight) i szerokość (outputWidth) - to podane w argumencie

    // Docelowe punkty na prostokącie o wymiarach outputWidth x outputHeight
    std::vector<cv::Point2f> dstCorners = {
        cv::Point2f(0, 0),                    // lewy górny
        cv::Point2f(outputWidth - 1, 0),     // prawy górny
        cv::Point2f(outputWidth - 1, outputHeight - 1), // prawy dolny
        cv::Point2f(0, outputHeight - 1)     // lewy dolny
    };

    // Obliczamy macierz transformacji perspektywiczej
    cv::Mat transform = cv::getPerspectiveTransform(corners, dstCorners);

    // Przekształcamy cały obraz - ale rozmiar wyjściowy dajemy taki, by pomieścić całą szerokość (np. szerokość oryginalnego obrazu lub większą)
    // Aby nie stracić części obrazu, możemy ustawić szerokość wyjściową na max (np. outputWidth + jakieś marginesy)
    // Albo najlepiej w tym wypadku: ustaw szerokość na np. outputWidth * 2 - jeśli nie jesteś pewien wymiarów

    // Dla prostoty – użyjemy szerokości oryginalnego obrazu (w pikselach) przeskalowanej proporcjonalnie do outputHeight
    double heightLeft = cv::norm(corners[3] - corners[0]);
    double heightRight = cv::norm(corners[2] - corners[1]);
    double avgHeight = (heightLeft + heightRight) / 2.0;

    double scale = outputHeight / avgHeight;
    int warpedWidth = static_cast<int>(originalImage.cols * scale);

    cv::Mat warpedFull;
    cv::warpPerspective(originalImage, warpedFull, transform, cv::Size(warpedWidth, outputHeight));

    // Wycinamy fragment zaczynający się od lewej strony przekształconego obrazu o szerokości outputWidth
    if (outputWidth > warpedFull.cols) {
        std::cout << "[Uwaga] Żądana szerokość wycięcia większa niż szerokość przekształconego obrazu. Zwracam cały obraz." << std::endl;
        return warpedFull;
    }

    cv::Rect cropRect(0, 0, outputWidth, outputHeight);
    cv::Mat cropped = warpedFull(cropRect).clone(); // klon, by mieć oddzielną macierz

    return cropped;
}




void showCornerPointsOnImage(const cv::Mat& originalImage,
                             const std::vector<Box>& boxes) {
    std::vector<cv::Point2f> corners = computeCornerPoints(boxes);
    if (corners.size() != 4) {
        std::cout << "[Błąd] Nie można wyświetlić rogów – niepoprawna liczba punktów." << std::endl;
        return;
    }

    cv::Mat display;
    originalImage.copyTo(display);

    std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 0, 255),   // TL
        cv::Scalar(0, 255, 0),   // TR
        cv::Scalar(255, 0, 0),   // BR
        cv::Scalar(0, 255, 255)  // BL
    };

    for (size_t i = 0; i < corners.size(); ++i) {
        cv::drawMarker(display, corners[i], colors[i], cv::MARKER_CROSS, 20, 2);
        cv::putText(display, "P" + std::to_string(i + 1),
                    corners[i] + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, colors[i], 2);
    }

    for (int i = 0; i < 4; ++i)
        cv::line(display, corners[i], corners[(i + 1) % 4], cv::Scalar(255, 255, 255), 1);

    cv::imshow("Corners on Original Image", display);
    cv::waitKey(10);
}


/*void setup(const std::string& zipFilePath) {
    zip* archive = nullptr;
    //std::vector<std::string> pngFiles;
    std::vector<PngEntry> pngBuffers;

    if (!loadZipAndListPNGs(zipFilePath, archive, pngBuffers)) {
        return;
    }

    // Załaduj pierwszy obraz, by znać rozmiar
    int currentImageIndex = 0;
    cv::Mat image = loadPngFromBuffer(pngBuffers, currentImageIndex);
    if (image.empty()) {
        zip_close(archive);
        return;
    }

    auto boxes = createBoxes(image.cols, image.rows, zipFilePath);
    int selectedBox = 1;

    while (true) {
        image = loadPngFromBuffer(pngBuffers, currentImageIndex);
        if (image.empty()) break;

        cv::Mat generatedImage = generateImage(image, boxes, selectedBox);
        displayImage(generatedImage);
        
        saveBoxPositionsToFile(zipFilePath, boxes, thresholdValue);
        displayBoxPositionsEvery5s(boxes);
        //showCirclesInBoxes(image, boxes, thresholdValue);// ...
        detectCirclesInBoxes(image, boxes, thresholdValue);
        showDetectedCircles(image, boxes, thresholdValue);

        std::vector<cv::Point2f> corners = computeCornerPoints(boxes);
        //showCornerPointsOnImage(image, boxes);

        if (corners.size() == 4) {
            cv::Mat warped = warpImageByCorners(image, corners, 1024);
            if (!warped.empty()) {
                cv::Mat resizedWarped;
                cv::resize(warped, resizedWarped, cv::Size((1024*1.5)/2, (1024)/2));

                cv::imshow("Warped", resizedWarped);
                cv::waitKey(10);
            }
        }
        
        

        char key = cv::waitKey(0);
        handleKeys(key, currentImageIndex, selectedBox, boxes, thresholdValue);
        
    }

    zip_close(archive);
}
*/

// ============================
// ZAMIENNIK setup(zipFilePath)
// ============================
void setup(const std::string& directoryPath) {

    if (!loadDirectoryPNGs(directoryPath, pngBuffers)) {
        return;
    }

    int currentImageIndex = 0;
    cv::Mat image = cv::imdecode(pngBuffers[currentImageIndex].data, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Nie udało się wczytać pierwszego obrazu.\n";
        return;
    }

    std::vector<std::vector<int>> masks = {
        {DARK_THRESHOLD}, 
        {BRIGHT_THRESHOLD}, 
        {BLUE_THRESHOLD}
    };
    
    // reszta identyczna jak wcześniej:
    auto boxes = createBoxes(image.cols, image.rows, directoryPath, masks);
    int selectedBox = 1;
    
//     std::cout << "a1: " << masks[0][0] << std::endl;
//     std::cout << "a2: " << masks[1][0] << std::endl;
//     std::cout << "a3: " << masks[2][0] << std::endl;

    while (true) {
        image = cv::imdecode(pngBuffers[currentImageIndex].data, cv::IMREAD_COLOR);
        if (image.empty()) break;

        cv::Mat generatedImage = generateImage(image, boxes, selectedBox);
        displayImage(generatedImage);
        
        saveBoxPositionsToFile(directoryPath, boxes, masks, thresholdValue);
        displayBoxPositionsEvery5s(boxes);
        
        detectCirclesInBoxes(image, boxes, thresholdValue);
        showDetectedCircles(image, boxes, thresholdValue);

        std::vector<cv::Point2f> corners = computeCornerPoints(boxes);
        if (corners.size() == 4) {
            cv::Mat warped = warpImageByCorners(image, corners, AREA_HEIGHT_PX);
            if (!warped.empty()) {
                //cv::Mat resizedWarped;
                //cv::resize(warped, resizedWarped, cv::Size((1024*1.75)/2, (1024)/2));

                // 🔹 Generowanie obrazu z krawędziami
                //cv::Mat highlighted = highlightBurnEdges(warped, 0.5, &boxes[4], selectedBox);
                cv::Mat fireBlob = highlightBurnEdges(warped, 0.5, &boxes[4], selectedBox, masks);

                // 🔹 Skalowanie, jeśli chcesz wyświetlić mniejszą wersję
                cv::Mat resizedBlob;
                cv::resize(fireBlob, resizedBlob, cv::Size((1024*1.75)/2, (1024)/2));

                imshowScaled("Fire blob", fireBlob);
                //imshowScaled("Warped with Burn Edges", warped);
                //cv::imshow("Fire Blob", fireBlob);
                //cv::imshow("Warped with Burn Edges", warped);
                
                Line hLine, vLine, cLine;
                findLongestLines(fireBlob, hLine, vLine);
                int blackPixelCount = countBlackPixels(fireBlob);
                cLine = findCentralLine(fireBlob);
                drawLines(warped, hLine, vLine, cLine);
                imshowScaled("Warped", warped);
                
                //std::cout << blackPixelCount << std::endl;
                //cv::waitKey(10);
                std::string header = generateHeader();
                std::string result = processResults(hLine, vLine, cLine, blackPixelCount, pngBuffers, currentImageIndex);
                std::cout << header << std::endl << result << std::endl;
            }
        }
        
        char key = cv::waitKey(0);
        handleKeys(key, currentImageIndex, selectedBox, boxes, thresholdValue, masks);
        
        // Korekta indeksu, jeśli wychodzi poza zakres
        if (currentImageIndex < 0)
            currentImageIndex = 0;
        else if (currentImageIndex >= static_cast<int>(pngBuffers.size()))
            currentImageIndex = static_cast<int>(pngBuffers.size()) - 1;
        }
}

// Funkcja zapisująca wyniki do pliku
void saveResultsToFile(const std::string& outputFileName, const std::string& header, const std::vector<std::string>& results) {
    std::ofstream outFile(outputFileName);
    if (!outFile.is_open()) {
        std::cerr << "Błąd przy otwieraniu pliku do zapisu: " << outputFileName << std::endl;
        return;
    }

    // Zapisujemy nagłówek
    outFile << header << std::endl;

    // Zapisujemy dane
    for (const auto& result : results) {
        outFile << result << std::endl;
    }

    outFile.close();
    std::cout << "Wyniki zapisane do pliku: " << outputFileName << std::endl;
}

// Funkcja analizująca obrazy i zapisująca wyniki
void analyse(const std::string& directoryPath, const std::string& outputFileName) {
    std::vector<PngEntry> pngBuffers;

    if (!loadDirectoryPNGs(directoryPath, pngBuffers)) {
        std::cerr << "Błąd przy ładowaniu PNG lub brak plików PNG w folderze.\n";
        return;
    }
    
    for (int currentImageIndex = 0; currentImageIndex < static_cast<int>(pngBuffers.size()); ++currentImageIndex) {
        if (pngBuffers[currentImageIndex].name.empty()) {
            std::cerr << "Nieprawidłowa nazwa pliku w pngBuffers dla obrazu " << currentImageIndex << ".\n";
            continue;  // Pomijamy ten obraz
        }

        cv::Mat image = cv::imdecode(pngBuffers[currentImageIndex].data, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Nie udało się wczytać obrazu " << pngBuffers[currentImageIndex].name << ".\n";
            continue;
        }
    }

    std::vector<std::string> results;  // Przechowuje wyniki do zapisania
    std::string header = generateHeader();
    if (header.empty()) {
        std::cerr << "Nagłówek jest pusty.\n";
        return;
    }
    
    for (int currentImageIndex = 0; currentImageIndex < static_cast<int>(pngBuffers.size()); ++currentImageIndex) {
        // Wczytaj obraz
        cv::Mat image = cv::imdecode(pngBuffers[currentImageIndex].data, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Nie udało się wczytać obrazu " << pngBuffers[currentImageIndex].name << ".\n";
            continue;
        }

        // Inicjalizowanie masek
        std::vector<std::vector<int>> masks = {
            {DARK_THRESHOLD},
            {BRIGHT_THRESHOLD},
            {BLUE_THRESHOLD}
        };

        // Tworzenie boxów
        auto boxes = createBoxes(image.cols, image.rows, directoryPath, masks);
        int selectedBox = 1;

        // Procesowanie obrazu
        cv::Mat generatedImage = generateImage(image, boxes, selectedBox);

        //saveBoxPositionsToFile(directoryPath, boxes, masks, thresholdValue);
        displayBoxPositionsEvery5s(boxes);

        detectCirclesInBoxes(image, boxes, thresholdValue);
        //showDetectedCircles(image, boxes, thresholdValue);

        std::vector<cv::Point2f> corners = computeCornerPoints(boxes);
        if (corners.size() == 4) {
            cv::Mat warped = warpImageByCorners(image, corners, AREA_HEIGHT_PX);
            if (!warped.empty()) {
                cv::Mat fireBlob = highlightBurnEdges(warped, 0.5, &boxes[4], selectedBox, masks);

                // Obliczanie linii i liczby czarnych pikseli
                Line hLine, vLine, cLine;
                findLongestLines(fireBlob, hLine, vLine);
                int blackPixelCount = countBlackPixels(fireBlob);
                cLine = findCentralLine(fireBlob);
                drawLines(warped, hLine, vLine, cLine);

                saveWarpedImage(
                    warped,
                    pngBuffers[currentImageIndex].name,
                    directoryPath
                );
                    
                // Przetwarzanie wyników
                std::string result = processResults(hLine, vLine, cLine, blackPixelCount, pngBuffers, currentImageIndex);
                results.push_back(result);  // Dodanie wyniku do listy wyników
            }
        }
    }

    // Zapisanie wyników do pliku
    saveResultsToFile(outputFileName, header, results);
}

// Główna funkcja
int main(int argc, char* argv[]) {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    
    if (argc < 2) {
        std::cerr << "Użycie: " << argv[0] << " [-s] <ścieżka do pliku ZIP>" << std::endl;
        return 1;
    }

    // Deklaracja zmiennych przed warunkiem
    std::string flag;
    std::string zipFilePath;

    // Sprawdzamy, czy jest tylko jeden argument (bez flagi)
    if (argc == 2) {
        flag = "";  // Brak flagi, ustawiamy pusty ciąg
        zipFilePath = argv[1];  // Ścieżka do pliku ZIP
    } else {
        flag = argv[1];  // Flaga (np. "-s")
        zipFilePath = argv[2];  // Ścieżka do pliku ZIP
    }
    
    // Sprawdzamy, czy ścieżka ma "/" na końcu i go usuwamy
    if (!zipFilePath.empty() && (zipFilePath.back() == '/' || zipFilePath.back() == '\\')) {
        zipFilePath.erase(zipFilePath.size() - 1);
    }
    
    if (flag == "-s") {
        setup(zipFilePath);
    } else {
        analyse_only = 1;
        analyse(zipFilePath, zipFilePath + ".csv");
    }

    return 0;
}
