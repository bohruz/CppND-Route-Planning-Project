#include <io2d.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>
#include "render.h"
#include "route_model.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string& path) {
  std::ifstream is{path, std::ios::binary | std::ios::ate};
  if (!is) return std::nullopt;

  auto size = is.tellg();
  std::vector<std::byte> contents(size);

  is.seekg(0);
  is.read((char*)contents.data(), size);

  if (contents.empty()) return std::nullopt;
  return std::move(contents);
}

int main(int argc, const char** argv) {
  std::string osm_data_file = "";
  if (argc > 1) {
    for (int i = 1; i < argc; ++i)
      if (std::string_view{argv[i]} == "-f" && ++i < argc)
        osm_data_file = argv[i];
  } else {
    std::cout << "To specify a map file use the following format: "
              << std::endl;
    std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
    osm_data_file = "../map.osm";
  }

  std::vector<std::byte> osm_data;

  if (osm_data.empty() && !osm_data_file.empty()) {
    std::cout << "Reading OpenStreetMap data from the following file: "
              << osm_data_file << std::endl;
    auto data = ReadFile(osm_data_file);
    if (!data)
      std::cout << "Failed to read." << std::endl;
    else
      osm_data = std::move(*data);
  }

  // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
  // user input for these values using std::cin. Pass the user input to the
  // RoutePlanner object below in place of 10, 10, 90, 90.

  // welcome message
  std::cout << "Hello, let's find your path? \n";

  // start variables
  float start_x = 0, start_y = 0;

  // end variables
  float end_x = 0, end_y = 0;

  // message to help the user
  std::cout << "Your start point is x = 10 and y = 10 and your end point is x "
               "= 90 and y = 90. \n"
            << "Please write these coordinates in the console.\n";

  // asking for the start point
  std::cout << "Start: ";
  std::cin >> start_x >> start_y;

  // asking for the end point
  std::cout << "End: ";
  std::cin >> end_x >> end_y;

  // printing the cordinates in the console
  std::cout << "Start point (" << start_x << ", " << start_y << ")\n";
  std::cout << "End point (" << end_x << ", " << end_y << ")\n";

  // Build Model.
  RouteModel model{osm_data};

  // Create RoutePlanner object and perform A* search.
  RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
  route_planner.AStarSearch();

  std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

  // Render results of search.
  Render render{model};

  auto display = io2d::output_surface{400,
                                      400,
                                      io2d::format::argb32,
                                      io2d::scaling::none,
                                      io2d::refresh_style::fixed,
                                      30};
  display.size_change_callback([](io2d::output_surface& surface) {
    surface.dimensions(surface.display_dimensions());
  });
  display.draw_callback(
      [&](io2d::output_surface& surface) { render.Display(surface); });
  display.begin_show();
}
