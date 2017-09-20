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
  uint32_t action;
};

struct ObstacleData : VisionData
{
  unsigned int model_id;
  unsigned int part_id;
  am2b_iface::ObstacleType type;
  double radius;
  std::vector<double> coeffs;
};

struct SurfaceData : VisionData
{
  unsigned int id;
  std::vector<double> normal;
  std::vector<double> vertices;
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
        std::shared_ptr<ObstacleData> od = std::make_shared<ObstacleData>();
        od->timestamp = last_stamp;

        while (tokens[i] != "}")
        {
          if (tokens[i] == "type")
          {
            if (tokens[i+1] == "Sphere")
            {
              od->type = am2b_iface::ObstacleType::Sphere;
            }
            else if (tokens[i+1] == "Capsule")
            {
              od->type = am2b_iface::ObstacleType::Capsule;
            }
            else if (tokens[i+1] == "Triangle")
            {
              od->type = am2b_iface::ObstacleType::Triangle;
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
            od->model_id = std::stoul(tokens[i+1].substr(0, split));
            od->part_id  = std::stoul(tokens[i+1].substr(split+1, tokens[i+1].length()-split-1));
            i++;
          }
          else if (tokens[i] == "action")
          {
            od->action = std::stoul(tokens[i+1]);
            i++;
          }
          else if (tokens[i] == "radius")
          {
            od->action = std::stod(tokens[i+1]);
            i++;
          }
          else if (tokens[i] == "Model Coefficients")
          {
            if (tokens[i+1] != "[")
              throw std::runtime_error("Failed to parse '" + filename + "': Expected vector of model coefficients. Found token: " + tokens[i+1]);

            i += 2; // skip opening brace
            while(tokens[i] != "]")
            {
              od->coeffs.push_back(std::stod(tokens[i]));
              i++;
            }
          }

          i++;
        }

        result.push_back(od);
      }
      else if (tokens[i] == "surface")
      {
        std::shared_ptr<SurfaceData> sd = std::make_shared<SurfaceData>();
        sd->timestamp = last_stamp;

        while (tokens[i] != "}")
        {
          if (tokens[i] == "id")
          {
            sd->id = std::stoul(tokens[i+1]);
            i++;
          }
          else if (tokens[i] == "action")
          {
            sd->action = std::stoul(tokens[i+1]);
            i++;
          }
          else if (tokens[i] == "normal")
          {
            if (tokens[i+1] != "[")
              throw std::runtime_error("Failed to parse '" + filename + "': Expected vector of surface normal coords. Found token: " + tokens[i+1]);

              i += 2; // skip opening brace
              while (tokens[i] != "]")
              {
                sd->normal.push_back(std::stod(tokens[i]));
                i++;
              }
          }
          else if (tokens[i] == "vertices")
          {
            if (tokens[i+1] != "[")
              throw std::runtime_error("Failed to parse '" + filename + "': Expected vector of surface vertices. Found token: " + tokens[i+1]);

              i += 2; // skip opening brace
              while (tokens[i] != "]")
              {
                sd->vertices.push_back(std::stod(tokens[i]));
                i++;
              }
          }
          i++;
        }

        result.push_back(sd);
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
