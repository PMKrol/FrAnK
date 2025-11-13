/*
 * g++ -std=c++17 -O2 FrAg.cpp -o frag
 * 
 * frag 25.11.07_14.37_OSB2
 * will cumulate 25.11.07_14.37_OSB2_frame.csv and 25.11.07_14.37_OSB2.csv (results of FrAnCs and FrIK).
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <iomanip>
#include <algorithm>

struct Record {
    long long Timestamp = 0;

    // Dane z OSB2_frame
    std::string MotorPosition;
    std::string WeightLoss;
    std::string IR1, IR2, IR3, IR4, IR5;

    // Dane z frame
    std::string hLine_px, vLine_px, cLine_px, blackPixelCount_px2;
    std::string hLine_cm, vLine_cm, cLine_cm, blackPixelCount_cm2;
};

std::vector<std::string> splitCSV(const std::string& line) {
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) result.push_back(item);
    return result;
}

std::vector<std::string> splitSpace(const std::string& line) {
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string item;
    while (ss >> item) result.push_back(item);
    return result;
}

// pomocnicza funkcja do łączenia danych (nie nadpisuje istniejących wartości pustymi)
void mergeRecords(Record& dest, const Record& src) {
    if (dest.MotorPosition.empty()) dest.MotorPosition = src.MotorPosition;
    if (dest.WeightLoss.empty()) dest.WeightLoss = src.WeightLoss;
    if (dest.IR1.empty()) dest.IR1 = src.IR1;
    if (dest.IR2.empty()) dest.IR2 = src.IR2;
    if (dest.IR3.empty()) dest.IR3 = src.IR3;
    if (dest.IR4.empty()) dest.IR4 = src.IR4;
    if (dest.IR5.empty()) dest.IR5 = src.IR5;
    if (dest.hLine_px.empty()) dest.hLine_px = src.hLine_px;
    if (dest.vLine_px.empty()) dest.vLine_px = src.vLine_px;
    if (dest.cLine_px.empty()) dest.cLine_px = src.cLine_px;
    if (dest.blackPixelCount_px2.empty()) dest.blackPixelCount_px2 = src.blackPixelCount_px2;
    if (dest.hLine_cm.empty()) dest.hLine_cm = src.hLine_cm;
    if (dest.vLine_cm.empty()) dest.vLine_cm = src.vLine_cm;
    if (dest.cLine_cm.empty()) dest.cLine_cm = src.cLine_cm;
    if (dest.blackPixelCount_cm2.empty()) dest.blackPixelCount_cm2 = src.blackPixelCount_cm2;
}

void readFrame(const std::string& filename, std::map<long long, Record>& records) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        std::cerr << "Nie można otworzyć pliku: " << filename << "\n";
        return;
    }

    std::string line;
    std::getline(f, line); // pomiń nagłówek

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto parts = splitCSV(line);
        if (parts.empty() || parts[0].empty()) continue;

        Record r;
        try { r.Timestamp = std::stoll(parts[0]); }
        catch (...) { continue; }

        if (parts.size() > 1) r.MotorPosition = parts[1];
        if (parts.size() > 2) r.WeightLoss = parts[2];
        if (parts.size() > 3) r.IR1 = parts[3];
        if (parts.size() > 4) r.IR2 = parts[4];
        if (parts.size() > 5) r.IR3 = parts[5];
        if (parts.size() > 6) r.IR4 = parts[6];
        if (parts.size() > 7) r.IR5 = parts[7];

        mergeRecords(records[r.Timestamp], r);
    }

    f.close();
}

void readBase(const std::string& filename, std::map<long long, Record>& records) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        std::cerr << "Nie można otworzyć pliku: " << filename << "\n";
        return;
    }

    std::string line;
    std::getline(f, line); // pomiń nagłówek

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto parts = splitSpace(line);
        if (parts.size() < 10) continue;

        Record r;
        try { r.Timestamp = std::stoll(parts[1]); }
        catch (...) { continue; }

        r.hLine_px = parts[2];
        r.vLine_px = parts[3];
        r.cLine_px = parts[4];
        r.blackPixelCount_px2 = parts[5];
        r.hLine_cm = parts[6];
        r.vLine_cm = parts[7];
        r.cLine_cm = parts[8];
        r.blackPixelCount_cm2 = parts[9];

        mergeRecords(records[r.Timestamp], r);
    }

    f.close();
}

void fillEmptyValues(std::map<long long, Record>& records) {
    Record prev;  // zapamiętuje ostatni pełny rekord

    for (auto& [ts, r] : records) {
        // jeśli brakuje wartości, uzupełnij z poprzedniego
        if (r.MotorPosition.empty()) r.MotorPosition = prev.MotorPosition;
        if (r.WeightLoss.empty())    r.WeightLoss    = prev.WeightLoss;
        if (r.IR1.empty())           r.IR1           = prev.IR1;
        if (r.IR2.empty())           r.IR2           = prev.IR2;
        if (r.IR3.empty())           r.IR3           = prev.IR3;
        if (r.IR4.empty())           r.IR4           = prev.IR4;
        if (r.IR5.empty())           r.IR5           = prev.IR5;

        if (r.hLine_px.empty())          r.hLine_px          = prev.hLine_px;
        if (r.vLine_px.empty())          r.vLine_px          = prev.vLine_px;
        if (r.cLine_px.empty())          r.cLine_px          = prev.cLine_px;
        if (r.blackPixelCount_px2.empty()) r.blackPixelCount_px2 = prev.blackPixelCount_px2;

        if (r.hLine_cm.empty())          r.hLine_cm          = prev.hLine_cm;
        if (r.vLine_cm.empty())          r.vLine_cm          = prev.vLine_cm;
        if (r.cLine_cm.empty())          r.cLine_cm          = prev.cLine_cm;
        if (r.blackPixelCount_cm2.empty()) r.blackPixelCount_cm2 = prev.blackPixelCount_cm2;

        // zaktualizuj "ostatni znany" rekord
        prev = r;
    }
}

void scaleMotorPosition(std::map<long long, Record>& records) {
    for (auto& [ts, r] : records) {
        if (r.MotorPosition.empty()) continue;  // pomiń puste

        try {
            // próbuj przekonwertować na liczbę całkowitą
            double value = std::stod(r.MotorPosition);
            value /= 1000.0;

            // zapisz z powrotem w postaci stringa z 3 miejscami po przecinku
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(3) << value;
            r.MotorPosition = ss.str();
        }
        catch (...) {
            // jeśli nie da się sparsować, pomiń bez błędu
            continue;
        }
    }
}

void scaleWeightLoss(std::map<long long, Record>& records) {
    for (auto& [ts, r] : records) {
        if (r.WeightLoss.empty()) continue;

        try {
            double value = std::stod(r.WeightLoss);
            value *= -10.0;

            std::ostringstream ss;
            ss << std::fixed << std::setprecision(3) << value;
            r.WeightLoss = ss.str();
        }
        catch (...) {
            continue;
        }
    }
}

void normalizeTimestamps(std::map<long long, Record>& records) {
    if (records.empty()) return;

    // najmniejszy timestamp (pierwszy klucz w mapie)
    long long firstTs = records.begin()->first;

    // nowa mapa na znormalizowane dane
    std::map<long long, Record> newRecords;

    for (auto& [ts, r] : records) {
        long long newTs = (ts - firstTs) / 1000; // odejmij i podziel przez 1000
        r.Timestamp = newTs;
        newRecords[newTs] = r;
    }

    records = std::move(newRecords);
}


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Użycie: " << argv[0] << " [csv common name (without .csv)]\n";
        return 1;
    }

    std::string baseName = argv[1];
    std::string file1 = baseName + "_frame.csv"; // pierwszy plik (oddzielany przecinkami)
    std::string file2 = baseName + ".csv";             // drugi plik (oddzielany spacjami)
    std::string outFile = baseName + "_output.csv";

    std::ifstream f1(file1);
    std::ifstream f2(file2);
    if (!f1.is_open() || !f2.is_open()) {
        std::cerr << "Nie można otworzyć jednego z plików wejściowych.\n";
        return 1;
    }

    std::map<long long, Record> records;

    // --- Czytanie OSB2_frame.csv ---
    /*std::string line;
    std::getline(f1, line); // pomiń nagłówek
    while (std::getline(f1, line)) {
        if (line.empty()) continue;
        auto parts = splitCSV(line);
        if (parts.empty() || parts[0].empty()) continue;

        Record r;
        try { r.Timestamp = std::stoll(parts[0]); } catch (...) { continue; }

        if (parts.size() > 1) r.MotorPosition = parts[1];
        if (parts.size() > 2) r.WeightLoss = parts[2];
        if (parts.size() > 3) r.IR1 = parts[3];
        if (parts.size() > 4) r.IR2 = parts[4];
        if (parts.size() > 5) r.IR3 = parts[5];
        if (parts.size() > 6) r.IR4 = parts[6];
        if (parts.size() > 7) r.IR5 = parts[7];

        mergeRecords(records[r.Timestamp], r);
    }
    f1.close();*/
    readFrame(file1, records);
    
    // --- Czytanie frame.csv ---
    /*std::getline(f2, line); // pomiń nagłówek
    while (std::getline(f2, line)) {
        if (line.empty()) continue;
        auto parts = splitSpace(line);
        if (parts.size() < 10) continue;

        Record r;
        try { r.Timestamp = std::stoll(parts[1]); } catch (...) { continue; }

        r.hLine_px = parts[2];
        r.vLine_px = parts[3];
        r.cLine_px = parts[4];
        r.blackPixelCount_px2 = parts[5];
        r.hLine_cm = parts[6];
        r.vLine_cm = parts[7];
        r.cLine_cm = parts[8];
        r.blackPixelCount_cm2 = parts[9];

        mergeRecords(records[r.Timestamp], r);
    }
    f2.close();*/
    readBase(file2, records);

    // Uzupełnianie pustych wartości na podstawie poprzednich
    fillEmptyValues(records);
    
    scaleMotorPosition(records);
    
    scaleWeightLoss(records);
    
    normalizeTimestamps(records);
    
    // --- Zapis do CSV ---
    std::ofstream out(outFile);
    out << "Timestamp,MotorPosition(k-rot),WeightLoss(*10g),IR1 (max),"
        //<< "IR2 (down),IR3 (up),IR4 (right),IR5 (left),"
        //<< "hLine(px),vLine(px),cLine(px),blackPixelCount(px^2),"
        << "hLine(cm),vLine(cm),cLine(cm),blackPixelCount(cm^2)\n";

    for (const auto& [ts, r] : records) {
        out << ts << ","
            << r.MotorPosition << "," << r.WeightLoss << "," << r.IR1 << "," 
            //<< r.IR2 << "," << r.IR3 << "," << r.IR4 << "," << r.IR5 << ","
            //<< r.hLine_px << "," << r.vLine_px << "," << r.cLine_px << "," << r.blackPixelCount_px2 << ","
            << r.hLine_cm << "," << r.vLine_cm << "," << r.cLine_cm << "," << r.blackPixelCount_cm2
            << "\n";
    }

    out.close();
    std::cout << "✅ Zapisano wynik do: " << outFile << "\n";
    return 0;
}
