#ifndef VISION_DATA_PARSER_H
#define VISION_DATA_PARSER_H

#include <iface_vision_msg.hpp>

#include <ios>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>

struct VisionData
{
  double   timestamp;
  am2b_iface::VisionMessage data;

  VisionData(am2b_iface::ObstacleMessage om, double t) : timestamp(t), data(om, (int)(t*10))
  {}

  VisionData(am2b_iface::SurfaceMessage sm, double t) : timestamp(t), data(sm, (int)(t*10))
  {}
};

// Provides some helpful methods for parsing the vision_data.txt
// produced by recording robot data with the marker tracker.
class VisionDataParser
{
private:
  static std::vector<std::string> Tokenize(std::ifstream& file)
  {
    char ch;
    std::string curr_tok;
    std::vector<std::string> tokens;

    bool parse_fieldnames = false;
    while (file >> std::noskipws >> ch)
    {
      // on comments, discard everything to end of current line
      if (ch == '#')
      {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
      else if (std::isspace(ch) && !parse_fieldnames)
      {
        if (curr_tok.length() > 0)
        {
          tokens.push_back(curr_tok);
          curr_tok = "";
        }
      }
      else if (std::isspace(ch) && parse_fieldnames && curr_tok.length() == 0)
      {
        continue;
      }
      else if (ch == ':' && parse_fieldnames)
      {
        if (curr_tok.length() > 0)
        {
          tokens.push_back(curr_tok);
          curr_tok = "";
        }
      }
      else if (ch == ',')
      {
        if (curr_tok.length() > 0)
        {
          tokens.push_back(curr_tok);
          curr_tok = "";
        }
      }
      else if (ch == '{')
      {
        if (curr_tok.length() > 0)
        {
          tokens.push_back(curr_tok);
          curr_tok = "";
        }
        tokens.push_back(std::string(1, ch));
        parse_fieldnames = true;
      }
      else if (ch == '[')
      {
        if (curr_tok.length() > 0)
        {
          tokens.push_back(curr_tok);
          curr_tok = "";
        }
        tokens.push_back(std::string(1, ch));
      }
      else if (ch == ']')
      {
        if (curr_tok.length() > 0)
        {
          tokens.push_back(curr_tok);
          curr_tok = "";
        }
        tokens.push_back(std::string(1, ch));
      }
      else if (ch == '\n')
      {
        if (curr_tok.length() > 0)
        {
          tokens.push_back(curr_tok);
          curr_tok = "";
        }
      }
      else if (ch == '}')
      {
        if (curr_tok.length() > 0)
        {
          tokens.push_back(curr_tok);
          curr_tok = "";
        }
        tokens.push_back(std::string(1, ch));
        parse_fieldnames = false;
      }
      else
      {
        curr_tok += ch;
      }
    }

    return tokens;
  }

public:
  static std::vector<std::shared_ptr<VisionData>> Parse(std::string filename)
  {
    std::vector<std::shared_ptr<VisionData>> result;
    std::ifstream input(filename);
    if (!input.is_open())
    {
      throw std::runtime_error("Could not open file: '" + filename + "'");
    }

    std::vector<std::string> tokens = Tokenize(input);

    double last_stamp;
    size_t i = 0;
    while (i < tokens.size())
    {
      if (tokens[i] == "obstacle")
      {
        am2b_iface::ObstacleMessage om;
        while (tokens[i] != "}")
        {
          if (tokens[i] == "type")
          {
            if (tokens[i+1] == "Sphere")
            {
              om.type = am2b_iface::ObstacleType::Sphere;
            }
            else if (tokens[i+1] == "Capsule")
            {
              om.type = am2b_iface::ObstacleType::Capsule;
            }
            else if (tokens[i+1] == "Triangle")
            {
              om.type = am2b_iface::ObstacleType::Triangle;
            }
            else
            {
              throw std::runtime_error("Failed to parse '" + filename + "': Invalid Obstacle Type: '" + tokens[i+1] + "'");
            }
            i++;
          }
          else if (tokens[i] == "id")
          {
            // find the split
            auto split = tokens[i+1].find("|");
            om.model_id = std::stoul(tokens[i+1].substr(0, split));
            om.part_id  = std::stoul(tokens[i+1].substr(split+1, tokens[i+1].length()-split-1));
            i++;
          }
          else if (tokens[i] == "action")
          {
            om.action = std::stoul(tokens[i+1], nullptr, 16);
            i++;
          }
          else if (tokens[i] == "radius")
          {
            om.radius = std::stod(tokens[i+1]);
            i++;
          }
          else if (tokens[i] == "surface")
          {
            om.surface = std::stoi(tokens[i+1]);
            i++;
          }
          else if (tokens[i] == "Model Coefficients")
          {
            if (tokens[i+1] != "[")
              throw std::runtime_error("Failed to parse '" + filename + "': Expected vector of model coefficients. Found token: " + tokens[i+1]);

            i += 2; // skip opening brace
            std::vector<double> shape_coeffs;
            while(tokens[i] != "]")
            {
              shape_coeffs.push_back(std::stod(tokens[i]));
              i++;
            }
            if (shape_coeffs.size() != 9)
              throw std::runtime_error("Failed to parse '" + filename + "': Expected 9 shape coefficients for obstacle. Found: " + std::to_string(shape_coeffs.size()));
            std::copy(shape_coeffs.begin(), shape_coeffs.end(), om.coeffs);
          }

          i++;
        }

        result.push_back(std::make_shared<VisionData>(om, last_stamp));
      }
      else if (tokens[i] == "surface")
      {
        am2b_iface::SurfaceMessage sm;

        while (tokens[i] != "}")
        {
          if (tokens[i] == "id")
          {
            sm.id = std::stoul(tokens[i+1]);
            i++;
          }
          else if (tokens[i] == "action")
          {
            sm.action = std::stoul(tokens[i+1], nullptr, 16);
            i++;
          }
          else if (tokens[i] == "normal")
          {
            if (tokens[i+1] != "[")
              throw std::runtime_error("Failed to parse '" + filename + "': Expected vector of surface normal coords. Found token: " + tokens[i+1]);

              i += 2; // skip opening brace
              std::vector<double> normal_coords;
              while (tokens[i] != "]")
              {
                normal_coords.push_back(std::stod(tokens[i]));
                i++;
              }
              if (normal_coords.size() != 3)
                throw std::runtime_error("Failed to parse '" + filename + "': Expected 3 values for surface normal. Found: " + std::to_string(normal_coords.size()));
              std::copy(normal_coords.begin(), normal_coords.end(), sm.normal);
          }
          else if (tokens[i] == "vertices")
          {
            if (tokens[i+1] != "[")
              throw std::runtime_error("Failed to parse '" + filename + "': Expected vector of surface vertices. Found token: " + tokens[i+1]);

              i += 2; // skip opening brace
              std::vector<double> vertex_coords;
              while (tokens[i] != "]")
              {
                vertex_coords.push_back(std::stod(tokens[i]));
                i++;
              }
              if (vertex_coords.size() != 24)
                throw std::runtime_error("Failed to parse '" + filename + "': Expected vector of surface vertex coords to contain 24 elements. Found: " + std::to_string(vertex_coords.size()));
              std::copy(vertex_coords.begin(), vertex_coords.end(), sm.vertices);
          }
          i++;
        }

        result.push_back(std::make_shared<VisionData>(sm, last_stamp));
      }
      else
      {
        last_stamp = std::stod(tokens[i]);
      }
      i++;
    }

    return result;
  }
};

#endif // VISION_DATA_PARSER_H
