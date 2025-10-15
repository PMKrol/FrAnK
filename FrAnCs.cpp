/*
 * g++ -std=c++17 -o results2csvs results2csvs.cpp && ./results2csv \[25.10.03\]\ test001.txt
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <optional>

struct FrameData {
    long long timestamp = 0;
    double motorPosition = std::numeric_limits<double>::quiet_NaN();
    double scaleWeight = std::numeric_limits<double>::quiet_NaN();
    double irValues[5] = { std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN() };
    int maxCol = -1;
    int maxRow = -1;
};

bool startsWith(const std::string& str, const std::string& prefix) {
    return str.compare(0, prefix.size(), prefix) == 0;
}

std::string changeExtensionToCSV(const std::string& filename) {
    size_t dotPos = filename.find_last_of('.');
    /*if (dotPos == std::string::npos) {
        return filename + ".csv";
    } else {
        return filename.substr(0, dotPos) + ".csv";
    }*/
    
    if (dotPos == std::string::npos) {
        return filename;
    } else {
        return filename.substr(0, dotPos);
    }
}

struct IRData {
    long long timestamp;
    double values[5];  // IR1 = max, IR2 = down, etc.
    int maxCol = -1;   // nowa kolumna
    int maxRow = -1;   // nowy wiersz
};

struct MotorData {
    long long timestamp;
    double position;
};

struct ScaleData {
    long long timestamp;
    double weight;
};

long long parseTimestamp(const std::string& line) {
    size_t posTimestampStart = line.find_first_of("0123456789");
    if (posTimestampStart == std::string::npos) return 0;
    size_t posTimestampEnd = line.find("--", posTimestampStart);
    if (posTimestampEnd == std::string::npos) posTimestampEnd = line.size();
    std::string tsStr = line.substr(posTimestampStart, posTimestampEnd - posTimestampStart);
    return std::stoll(tsStr);
}

void processIRBlock(std::ifstream& infile, std::vector<IRData>& irData, const std::string& headerLine) {
    const int width = 16;
    const int height = 12;

    long long timestamp = parseTimestamp(headerLine);
    std::vector<double> pixels;
    std::string line;

    while (std::getline(infile, line)) {
        if (line.find("-- IR Stop --") != std::string::npos) break;
        std::istringstream iss(line);
        double val;
        while (iss >> val) pixels.push_back(val);
    }

    if (pixels.empty()) return;

    double maxVal = pixels[0];
    size_t maxIndex = 0;
    for (size_t i = 1; i < pixels.size(); ++i) {
        if (pixels[i] > maxVal) {
            maxVal = pixels[i];
            maxIndex = i;
        }
    }

    int maxRow = maxIndex / width;
    int maxCol = maxIndex % width;

    auto getPixel = [&](int row, int col) -> double {
        if (row < 0 || row >= height || col < 0 || col >= width) return 0.0;
        return pixels[row * width + col];
    };

    IRData data;
    data.timestamp = timestamp;
    data.values[0] = maxVal;
    data.values[1] = getPixel(maxRow, maxCol - 3);
    data.values[2] = getPixel(maxRow, maxCol + 3);
    data.values[3] = getPixel(maxRow + 3, maxCol);
    data.values[4] = getPixel(maxRow - 3, maxCol);
    data.maxCol = maxCol;
    data.maxRow = maxRow;

    irData.push_back(data);
}



void processScaleBlock(std::ifstream& infile, std::vector<ScaleData>& scaleData, const std::string& headerLine) {
    long long timestamp = parseTimestamp(headerLine);

    double weight = std::numeric_limits<double>::quiet_NaN();
    std::string line;
    while (std::getline(infile, line)) {
        if (line.find("Weight:") != std::string::npos) {
            std::istringstream iss(line);
            std::string word;
            iss >> word;
            iss >> weight;
            break;
        }
        if (line.find("-- Scale Stop --") != std::string::npos) break;
    }

    scaleData.push_back({ timestamp, weight });
}

void processMotorBlock(std::ifstream& infile, std::vector<MotorData>& motorData, const std::string& headerLine) {
    long long timestamp = parseTimestamp(headerLine);

    double position = std::numeric_limits<double>::quiet_NaN();
    std::string line;
    while (std::getline(infile, line)) {
        if (line.find("Position:") != std::string::npos) {
            std::istringstream iss(line);
            std::string word;
            iss >> word;
            iss >> position;
            break;
        }
        if (line.find("-- Motor Stop --") != std::string::npos) break;
    }

    motorData.push_back({ timestamp, position });
}

// ==========================
// === ZAPIS CSV #1: Default
// ==========================
void writeCSV_Separate(const std::string& outputFile,
                      const std::vector<IRData>& irData,
                      const std::vector<MotorData>& motorData,
                      const std::vector<ScaleData>& scaleData) {

    std::ofstream outfile(outputFile);
    if (!outfile.is_open()) {
        std::cerr << "Nie można zapisać do pliku: " << outputFile << std::endl;
        return;
    }

    outfile << "Timestamp_IR,IR1 (max),IR2 (down),IR3 (up),IR4 (right),IR5 (left),,Timestamp_Motor,MotorPosition,,Timestamp_Scale,ScaleWeight\n";

    size_t maxRows = std::max({ irData.size(), motorData.size(), scaleData.size() });

    for (size_t i = 0; i < maxRows; ++i) {
        if (i < irData.size()) {
            outfile << irData[i].timestamp;
            for (int j = 0; j < 5; ++j) outfile << "," << irData[i].values[j];
        } else {
            outfile << ",,,,,";
        }

        outfile << ",,";

        if (i < motorData.size()) {
            outfile << motorData[i].timestamp << "," << motorData[i].position;
        } else {
            outfile << ",";
        }

        outfile << ",,";

        if (i < scaleData.size()) {
            outfile << scaleData[i].timestamp << "," << scaleData[i].weight;
        } else {
            outfile << ",,";
        }

        outfile << "\n";
    }
}

// ===============================
// === ZAPIS CSV #2: FrameData style
// ===============================
void writeCSV_FrameDataStyle(const std::string& outputFile,
                             const std::vector<IRData>& irData,
                             const std::vector<MotorData>& motorData,
                             const std::vector<ScaleData>& scaleData) {
    std::ofstream outfile(outputFile);
    if (!outfile.is_open()) {
        std::cerr << "Nie można zapisać do pliku: " << outputFile << std::endl;
        return;
    }

    size_t i_ir = 0, i_motor = 0, i_scale = 0;
    const size_t n_ir = irData.size();
    const size_t n_motor = motorData.size();
    const size_t n_scale = scaleData.size();
    
    outfile << "Timestamp,MotorPosition,WeightLoss,IR1 (max),IR2 (down),IR3 (up),IR4 (right),IR5 (left)\n";


    // Używamy wskaźników i porównujemy timestampy, żeby wybrać najmniejszy kolejny
    while (i_ir < n_ir || i_motor < n_motor || i_scale < n_scale) {
        // Znajdź minimalny timestamp spośród aktualnych pozycji (jeśli istnieją)
        double ts_ir = (i_ir < n_ir) ? irData[i_ir].timestamp : std::numeric_limits<double>::infinity();
        double ts_motor = (i_motor < n_motor) ? motorData[i_motor].timestamp : std::numeric_limits<double>::infinity();
        double ts_scale = (i_scale < n_scale) ? scaleData[i_scale].timestamp : std::numeric_limits<double>::infinity();

        // Minimalny timestamp i skąd pochodzi
        double min_ts = std::min({ts_ir, ts_motor, ts_scale});

        FrameData frame;
        frame.timestamp = min_ts;

        // Wypełniamy dane z tych źródeł, które mają timestamp równy min_ts
        // i przesuwamy wskaźnik w tym wektorze
        if (i_ir < n_ir && irData[i_ir].timestamp == min_ts) {
            for (int j = 0; j < 5; ++j) {
                frame.irValues[j] = irData[i_ir].values[j];
            }
            ++i_ir;
        } else {
            // Jeśli nie ma danych IR dla tego timestampu, ustawiamy NaN lub coś podobnego
            for (int j = 0; j < 5; ++j) {
                frame.irValues[j] = std::numeric_limits<double>::quiet_NaN();
            }
        }

        if (i_motor < n_motor && motorData[i_motor].timestamp == min_ts) {
            frame.motorPosition = motorData[i_motor].position;
            ++i_motor;
        } else {
            frame.motorPosition = std::numeric_limits<double>::quiet_NaN();
        }

        if (i_scale < n_scale && scaleData[i_scale].timestamp == min_ts) {
            frame.scaleWeight = scaleData[i_scale].weight;
            ++i_scale;
        } else {
            frame.scaleWeight = std::numeric_limits<double>::quiet_NaN();
        }

        // Zapisujemy ramkę do pliku
        outfile << frame.timestamp << ",";

        if (std::isnan(frame.motorPosition))
            outfile << ",";
        else
            outfile << frame.motorPosition << ",";

        if (std::isnan(frame.scaleWeight))
            outfile << ",";
        else
            outfile << frame.scaleWeight << ",";

        if (std::isnan(frame.irValues[0]))
            outfile << ",,,,\n";
        else
            outfile << frame.irValues[0] << "," << frame.irValues[1] << "," << frame.irValues[2]
                    << "," << frame.irValues[3] << "," << frame.irValues[4] << "\n";
    }
}


// =========================
// === ZAPIS CSV #3: Interpolate (wypełnione brakujące dane)
// =========================
// Interpolacja liniowa
double interpolate(double t, double t0, double v0, double t1, double v1) {
    if (t1 == t0) return v0;
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0);
}

// Funkcja do uzupełniania braków NaN poprzez liniową interpolację
// Przeszukuje wektor frameów i uzupełnia kolumnę wskazaną przez getter/setter
template<typename Getter, typename Setter>
// Funkcja interpolująca dla pojedynczej kolumny (Motor, Scale)
void interpolateColumn(std::vector<FrameData>& frames, Getter getter, Setter setter) {
    size_t n = frames.size();
    size_t firstKnown = 0;
    while (firstKnown < n && std::isnan(getter(frames[firstKnown]))) {
        ++firstKnown;
    }
    for (size_t i = 0; i < firstKnown; ++i) {
        setter(frames[i], 0.0);
    }
    if (firstKnown == n) {
        for (auto& f : frames) setter(f, 0.0);
        return;
    }

    size_t lastKnown = firstKnown;
    for (size_t i = firstKnown + 1; i <= n; ++i) {
        if (i == n || !std::isnan(getter(frames[i]))) {
            double t0 = frames[lastKnown].timestamp;
            double v0 = getter(frames[lastKnown]);
            double t1 = (i == n) ? t0 : frames[i].timestamp;
            double v1 = (i == n) ? v0 : getter(frames[i]);
            for (size_t j = lastKnown + 1; j < i; ++j) {
                double t = frames[j].timestamp;
                double val = interpolate(t, t0, v0, t1, v1);
                setter(frames[j], val);
            }
            lastKnown = i;
        }
    }
}


// Funkcja interpolująca pojedynczy kanał IR
void interpolateIRColumn(std::vector<FrameData>& frames, int channel) {
    size_t n = frames.size();
    // Znajdź pierwszy znany indeks (nie NaN)
    size_t firstKnown = 0;
    while (firstKnown < n && std::isnan(frames[firstKnown].irValues[channel])) {
        ++firstKnown;
    }
    for (size_t i = 0; i < firstKnown; ++i) {
        frames[i].irValues[channel] = 0.0;
    }
    if (firstKnown == n) {
        for (auto& f : frames) f.irValues[channel] = 0.0;
        return;
    }

    size_t lastKnown = firstKnown;
    for (size_t i = firstKnown + 1; i <= n; ++i) {
        if (i == n || !std::isnan(frames[i].irValues[channel])) {
            double t0 = frames[lastKnown].timestamp;
            double v0 = frames[lastKnown].irValues[channel];
            double t1 = (i == n) ? t0 : frames[i].timestamp;
            double v1 = (i == n) ? v0 : frames[i].irValues[channel];
            for (size_t j = lastKnown + 1; j < i; ++j) {
                double t = frames[j].timestamp;
                frames[j].irValues[channel] = v0 + (v1 - v0) * (t - t0) / (t1 - t0);
            }
            lastKnown = i;
        }
    }
}


void writeCSV_Interpolate(const std::string& outputFile,
                          const std::vector<IRData>& irData,
                          const std::vector<MotorData>& motorData,
                          const std::vector<ScaleData>& scaleData) {
    std::ofstream outfile(outputFile);
    if (!outfile.is_open()) {
        std::cerr << "Nie można zapisać do pliku: " << outputFile << std::endl;
        return;
    }

    size_t n_ir = irData.size();
    size_t n_motor = motorData.size();
    size_t n_scale = scaleData.size();

    outfile << "Timestamp,MotorPosition [k],WeightLoss [mg],IR1 (max),IR2 (down),IR3 (up),IR4 (right),IR5 (left)\n";

    // 1) Zbierz wszystkie unikalne timestampy
    std::vector<double> all_timestamps;
    all_timestamps.reserve(n_ir + n_motor + n_scale);

    for (const auto& d : irData) all_timestamps.push_back(d.timestamp);
    for (const auto& d : motorData) all_timestamps.push_back(d.timestamp);
    for (const auto& d : scaleData) all_timestamps.push_back(d.timestamp);

    std::sort(all_timestamps.begin(), all_timestamps.end());
    all_timestamps.erase(std::unique(all_timestamps.begin(), all_timestamps.end()), all_timestamps.end());

    // 2) Inicjuj ramki
    std::vector<FrameData> frames(all_timestamps.size());
    for (size_t i = 0; i < all_timestamps.size(); ++i) {
        frames[i].timestamp = all_timestamps[i];
        frames[i].motorPosition = std::numeric_limits<double>::quiet_NaN();
        frames[i].scaleWeight = std::numeric_limits<double>::quiet_NaN();
        for (int j = 0; j < 5; ++j)
            frames[i].irValues[j] = std::numeric_limits<double>::quiet_NaN();
    }

    // Wypełnij IR
    size_t idx_ir = 0;
    for (auto& frame : frames) {
        while (idx_ir < n_ir && irData[idx_ir].timestamp < frame.timestamp) ++idx_ir;
        if (idx_ir < n_ir && irData[idx_ir].timestamp == frame.timestamp) {
            for (int j = 0; j < 5; ++j)
                frame.irValues[j] = irData[idx_ir].values[j];
            
            frame.maxCol = irData[idx_ir].maxCol;
            frame.maxRow = irData[idx_ir].maxRow;
            ++idx_ir;
        }
    }

    // Wypełnij Motor
    size_t idx_motor = 0;
    for (auto& frame : frames) {
        while (idx_motor < n_motor && motorData[idx_motor].timestamp < frame.timestamp) ++idx_motor;
        if (idx_motor < n_motor && motorData[idx_motor].timestamp == frame.timestamp) {
            frame.motorPosition = motorData[idx_motor].position;
            ++idx_motor;
        }
    }

    // Wypełnij Scale
    size_t idx_scale = 0;
    for (auto& frame : frames) {
        while (idx_scale < n_scale && scaleData[idx_scale].timestamp < frame.timestamp) ++idx_scale;
        if (idx_scale < n_scale && scaleData[idx_scale].timestamp == frame.timestamp) {
            frame.scaleWeight = scaleData[idx_scale].weight;
            ++idx_scale;
        }
    }

    // 3) Interpoluj brakujące wartości
    interpolateColumn(frames,
        [](const FrameData& f) { return f.motorPosition; },
        [](FrameData& f, double v) { f.motorPosition = v; });

    interpolateColumn(frames,
        [](const FrameData& f) { return f.scaleWeight; },
        [](FrameData& f, double v) { f.scaleWeight = v; });

    for (int j = 0; j < 5; ++j) {
        interpolateIRColumn(frames, j);
    }

    // 4) Zaokrąglanie do 2 miejsc po przecinku
    auto round2 = [](double val) {
        return std::round(val * 100.0) / 100.0;
    };

    for (auto& f : frames) {
        f.motorPosition = round2(f.motorPosition)/1000;
        f.scaleWeight = round2(f.scaleWeight)*-100;
        for (int j = 0; j < 5; ++j)
            f.irValues[j] = round2(f.irValues[j]);
    }

    // 5) Zapis do pliku
    for (const auto& f : frames) {
        outfile << std::fixed << std::setprecision(2) << f.timestamp << ",";
        outfile << f.motorPosition << ",";
        outfile << f.scaleWeight << ",";
        for (int j = 0; j < 5; ++j) {
            outfile << f.irValues[j] << ",";
            }
            outfile << f.maxCol << "," << f.maxRow << "\n";
        }
}

// ===========================
// === Parsowanie i główna logika
// ===========================
void parseFile(const std::string& inputFile) {
    std::ifstream infile(inputFile);
    if (!infile.is_open()) {
        std::cerr << "Nie można otworzyć pliku wejściowego: " << inputFile << std::endl;
        return;
    }

    std::vector<IRData> irData;
    std::vector<MotorData> motorData;
    std::vector<ScaleData> scaleData;

    std::string line;

    while (std::getline(infile, line)) {
        if (startsWith(line, "-- IR Start --")) {
            processIRBlock(infile, irData, line);
        } else if (startsWith(line, "-- Scale Start --")) {
            processScaleBlock(infile, scaleData, line);
        } else if (startsWith(line, "-- Motor Start --")) {
            processMotorBlock(infile, motorData, line);
        }
    }

    infile.close();

    // Zapis #1 – jak dotychczas
    writeCSV_Separate(changeExtensionToCSV(inputFile) + "_separate.csv", irData, motorData, scaleData);

    // Zapis #2 – FrameData
    writeCSV_FrameDataStyle(changeExtensionToCSV(inputFile) + "_frame.csv", irData, motorData, scaleData);

    // Zapis #3 – pusty plik
    writeCSV_Interpolate(changeExtensionToCSV(inputFile) + "_interpolate.csv", irData, motorData, scaleData);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Użycie: " << argv[0] << " inputfile.txt" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    parseFile(inputFile);

    return 0;
}


