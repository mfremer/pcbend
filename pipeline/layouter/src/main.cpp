#include <Sheet.hpp>
#include <Layout.hpp>

#include <filesystem>
#include <iostream>
#include <optional>

#include <cstdlib>

std::string extract_name_from_path(const std::string& path) {
    size_t last_slash = path.find_last_of('/');
    size_t last_dot = path.find_last_of('.');
    return path.substr(last_slash + 1, last_dot - last_slash - 1);
}

void do_things(
    const std::string& file_stg, const std::string& file_sht,
    const std::string& file_fis, const std::string& dir_svgs)
{
    std::string pcb_name = extract_name_from_path(file_fis);
    std::string dir_svgs_final = dir_svgs;
    if (dir_svgs.back() != '/') { dir_svgs_final.push_back('/'); }
    std::string dir_path = dir_svgs_final; // + pcb_name;

    if (!std::filesystem::exists(dir_path)) {
        std::filesystem::create_directory(dir_path);
    }
    
    Layout layout(file_stg, file_sht, file_fis);

    layout.compute_layout();
    layout.write_layout(dir_path + pcb_name);
    layout.write_auxiliary_files(dir_path + pcb_name);
    layout.print_info();
}

int main(int argc, char* argv[]) {
    std::string file_stg{};
    std::string file_sht{};
    std::string file_fis{};
    std::string dir_svgs{};

    if (argc == 5) {
        file_stg = argv[1];
        file_sht = argv[2];
        file_fis = argv[3];
        dir_svgs = argv[4];
    } else {
        std::cerr << "Wrong number of arguments\n";
        std::cerr << "./Layout2SVG <in:*.sheet> <in:*.trc> [LED]<in:*.module> [CON]<in:*.module> <out:svgs/>\n";
        exit(EXIT_FAILURE);
    }

    do_things(file_stg, file_sht, file_fis, dir_svgs);

    return EXIT_SUCCESS;
}