#include <Sheet.hpp>
#include <parsing_helpers.hpp>

#include <cstdlib>

std::string strip_extension(const std::string& filename) {
    size_t dot_pos = filename.find_last_of('.');
    return filename.substr(0, dot_pos);
}

std::string interface() {
    return "./Cutter inp:<*.cfg> inp:<*.sheet> out:<*.fis> inp:<\"connSFIds\">\n";
}

void print_info(
    const std::string& file_stg,
    const std::string& file_sht,
    const std::string& file_out,
    const std::vector<int>& connector_sfids)
{
    std::cout << "CUTTER\n";
    std::cout << "file_stg\t" << file_stg << '\n';
    std::cout << "file_sht\t" << file_sht << '\n';
    std::cout << "file_out\t" << file_out << '\n';

    std::cout << "conSFIDs\t";
    if (connector_sfids.empty()) { std::cout << "default"; }
    else { for (auto fid : connector_sfids) { std::cout << fid << ' '; } }
    std::cout << '\n';
}

int main(int argc, char* argv[]) {
    // Input initialization
    std::string file_stg{};
    std::string file_sht{};
    std::string file_out{};
    std::vector<int> connector_sfids{}; // should be int
    bool split = false;

    // Input handling
    if (argc == 4) {
        file_stg = argv[1];
        file_sht = argv[2];
        file_out = argv[3];
        // connector_sfids empty, default connectors
    } else if (argc == 5) {
        file_stg = argv[1];
        file_sht = argv[2];
        file_out = argv[3];
        connector_sfids = string_to_vector<int>(argv[4]);
    } else if (argc == 6) {
        file_stg = argv[1];
        file_sht = argv[2];
        file_out = argv[3];
        connector_sfids = string_to_vector<int>(argv[4]);
        split = std::stoi(argv[5]);
    } else {
        std::cerr << "[ERROR] Wrong number of arguments\n";
        std::cerr << interface();
        exit(EXIT_FAILURE);
    }
    print_info(file_stg, file_sht, file_out, connector_sfids);

    // Output information
    // std::string file_out = strip_extension(file_sht) + ".fis";

    // Generate faces file
    Sheet sh;
    sh.use_settings(file_stg);
    sh.read_sheet(file_sht);
    sh.init_sheet();

    auto [hid, max] = sh.half_hinges_max_shrink();
    std::cout << "Max half-hinge shrinkage at HID " << hid << "\tby " << max << "mm\n";

    sh.write_faces(file_out, connector_sfids, split);

    return EXIT_SUCCESS;
}
