#include "map.h"

bool Map::get_map(const char* FileName)
{
    tinyxml2::XMLElement *root = nullptr;
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (root)
    {
        map_is_roadmap = false;
        
    }
    else
    {
        map_is_roadmap = true;
        return get_roadmap(FileName);
    }
}

double Map::get_i(int id) const
{
    if(!map_is_roadmap)
        return int(id/width);
    else
        return nodes[id].i;
}

double Map::get_j(int id) const
{
    if(!map_is_roadmap)
        return int(id%width);
    else
        return nodes[id].j;
}

bool Map::get_grid(const nav2_costmap_2d::Costmap2D* costmap)
{
    map_is_roadmap = false;

    height = costmap->getSizeInCellsY();
    width = costmap->getSizeInCellsX();
    std::cout << height << " " << width << std::endl;
    grid.resize(width);
    for (int i = 0; i < width; ++i)
        grid[i].resize(height);
    unsigned int mx, my;
    size = (width * height);
    for (int i = 0; i < size; i++)
    {
        costmap->indexToCells(i, mx, my);

       grid[mx][my] = costmap->getCost(mx, my) > 0 ? CN_OBSTL : 0;

        //std::cout << i << " " << mx << " " << my << " " << grid[mx][my] << std::endl;

    }

//    for (int i  = 0; i < height; i++)
//    {
//        for (int j = 0; j < width; j++)
//        {
//          
//            grid[i][j] = value;
//
//        }
//    }
    std::vector<Step> moves;
    valid_moves.resize(height*width);
    if(connectedness == 2)
        moves = {{0,1}, {1,0}, {-1,0},  {0,-1}};
    else if(connectedness == 3)
        moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1}};
    else if(connectedness == 4)
        moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1},
                 {1,2}, {2,1}, {2,-1}, {1,-2}, {-1,-2}, {-2,-1}, {-2,1},  {-1,2}};
    else
        moves = {{0,1},   {1,1},   {1,0},   {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1},
                 {1,2},   {2,1},   {2,-1},  {1,-2},  {-1,-2}, {-2,-1}, {-2,1}, {-1,2},
                 {1,3},   {2,3},   {3,2},   {3,1},   {3,-1},  {3,-2},  {2,-3}, {1,-3},
                 {-1,-3}, {-2,-3}, {-3,-2}, {-3,-1}, {-3,1},  {-3,2},  {-2,3}, {-1,3}};
    for(int i = 0; i < height; i++)
        for(int j = 0; j < width; j++)
        {
            std::vector<bool> valid(moves.size(), true);
            for(unsigned int k = 0; k < moves.size(); k++)
                if((i + moves[k].i) < 0 || (i + moves[k].i) >= height || (j + moves[k].j) < 0 || (j + moves[k].j) >= width
                        || cell_is_obstacle(i + moves[k].i, j + moves[k].j)
                        || !check_line(i, j, i + moves[k].i, j + moves[k].j))
                    valid[k] = false;
            std::vector<Node> v_moves = {};
            for(unsigned int k = 0; k < valid.size(); k++)
                if(valid[k])
                    v_moves.push_back(Node((i + moves[k].i)*width + moves[k].j + j, 0, 0, i + moves[k].i, j + moves[k].j));
            valid_moves[i*width+j] = v_moves;
        }
//    std::cout << "Map " << __LINE__ << std::endl;
    return true;
}

bool Map::get_grid(const nav_msgs::msg::OccupancyGrid& grid_)
{

    tinyxml2::XMLElement *root = nullptr, *map = nullptr, *element = nullptr, *mapnode = nullptr;
   
    height = grid_.info.height;
    width = grid_.info.width;
    grid.resize(height);
    for (int i = 0; i < height; ++i)
            grid[i].resize(width);
    
    int grid_count = 0;
    int value = 0;
    for (int i  = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
          
            grid[i][j] = value;
            std::cout<< "i:" << i <<" j: " << j << "value: "<< value<<std::endl;
            grid_count++;

        }
    }
    
    size = width*height;
    std::vector<Step> moves;
    valid_moves.resize(height*width);
    if(connectedness == 2)
        moves = {{0,1}, {1,0}, {-1,0},  {0,-1}};
    else if(connectedness == 3)
        moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1}};
    else if(connectedness == 4)
        moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1},
                 {1,2}, {2,1}, {2,-1}, {1,-2}, {-1,-2}, {-2,-1}, {-2,1},  {-1,2}};
    else
        moves = {{0,1},   {1,1},   {1,0},   {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1},
                 {1,2},   {2,1},   {2,-1},  {1,-2},  {-1,-2}, {-2,-1}, {-2,1}, {-1,2},
                 {1,3},   {2,3},   {3,2},   {3,1},   {3,-1},  {3,-2},  {2,-3}, {1,-3},
                 {-1,-3}, {-2,-3}, {-3,-2}, {-3,-1}, {-3,1},  {-3,2},  {-2,3}, {-1,3}};
    for(int i = 0; i < height; i++)
        for(int j = 0; j < width; j++)
        {
            std::vector<bool> valid(moves.size(), true);
            for(unsigned int k = 0; k < moves.size(); k++)
                if((i + moves[k].i) < 0 || (i + moves[k].i) >= height || (j + moves[k].j) < 0 || (j + moves[k].j) >= width
                        || cell_is_obstacle(i + moves[k].i, j + moves[k].j)
                        || !check_line(i, j, i + moves[k].i, j + moves[k].j))
                    valid[k] = false;
            std::vector<Node> v_moves = {};
            for(unsigned int k = 0; k < valid.size(); k++)
                if(valid[k])
                    v_moves.push_back(Node((i + moves[k].i)*width + moves[k].j + j, 0, 0, i + moves[k].i, j + moves[k].j));
            valid_moves[i*width+j] = v_moves;
        }
    return true;
}

bool Map::get_roadmap(const char *FileName)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    tinyxml2::XMLElement *root = 0, *element = 0, *data;
    std::string value;
    std::stringstream stream;
    root = doc.FirstChildElement("graphml")->FirstChildElement("graph");
    for(element = root->FirstChildElement("node"); element; element = element->NextSiblingElement("node"))
    {
        data = element->FirstChildElement();

        stream.str("");
        stream.clear();
        stream << data->GetText();
        stream >> value;
        auto it = value.find_first_of(",");
        stream.str("");
        stream.clear();
        stream << value.substr(0, it);
        double i;
        stream >> i;
        stream.str("");
        stream.clear();
        value.erase(0, ++it);
        stream << value;
        double j;
        stream >> j;
        gNode node;
        node.i = i;
        node.j = j;
        nodes.push_back(node);
    }
    for(element = root->FirstChildElement("edge"); element; element = element->NextSiblingElement("edge"))
    {
        std::string source = std::string(element->Attribute("source"));
        std::string target = std::string(element->Attribute("target"));
        source.erase(source.begin(),++source.begin());
        target.erase(target.begin(),++target.begin());
        int id1, id2;
        stream.str("");
        stream.clear();
        stream << source;
        stream >> id1;
        stream.str("");
        stream.clear();
        stream << target;
        stream >> id2;
        nodes[id1].neighbors.push_back(id2);
    }
    for(gNode cur:nodes)
    {
        Node node;
        std::vector<Node> neighbors;
        neighbors.clear();
        for(unsigned int i = 0; i < cur.neighbors.size(); i++)
        {
            node.i = nodes[cur.neighbors[i]].i;
            node.j = nodes[cur.neighbors[i]].j;
            node.id = cur.neighbors[i];
            neighbors.push_back(node);
        }
        valid_moves.push_back(neighbors);
    }
    size = int(nodes.size());
    return true;
}

void Map::print_map()
{
    std::cout<<height<<"x"<<width<<std::endl;
    for(int i = 0; i < height; i++)
    {
        std::cout<<"<row>";
        for(int j = 0; j < width; j++)
            std::cout<<grid[i][j]<<" ";
        std::cout<<"</row>"<<std::endl;
    }
}

void Map::printPPM()
{
    std::cout<<"P3\n"<<width<<" "<<height<<"\n255\n";
    for(int i = 0; i < height; i++)
        for(int j = 0; j < width; j++)
        {
            if(grid[i][j]==1)
                std::cout<<"0 0 0\n";
            else
                std::cout<<"255 255 255\n";
        }
}


bool Map::cell_is_obstacle(int i, int j) const
{
    return (grid[i][j] == CN_OBSTL);
}

std::vector<Node> Map::get_valid_moves(int id) const
{
    return valid_moves[id];
}

bool Map::check_line(int x1, int y1, int x2, int y2)
{
    int delta_x(std::abs(x1 - x2));
    int delta_y(std::abs(y1 - y2));
    if((delta_x > delta_y && x1 > x2) || (delta_y >= delta_x && y1 > y2))
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    int step_x(x1 < x2 ? 1 : -1);
    int step_y(y1 < y2 ? 1 : -1);
    int error(0), x(x1), y(y1);
    int gap = int(agent_size*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON);
    int k, num;

    if(delta_x > delta_y)
    {
        int extraCheck = int(agent_size*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON);
        for(int n = 1; n <= extraCheck; n++)
        {
            error += delta_y;
            num = (gap - error)/delta_x;
            for(k = 1; k <= num; k++)
                if(cell_is_obstacle(x1 - n*step_x, y1 + k*step_y))
                    return false;
            for(k = 1; k <= num; k++)
                if(cell_is_obstacle(x2 + n*step_x, y2 - k*step_y))
                    return false;
        }
        error = 0;
        for(x = x1; x != x2 + step_x; x++)
        {
            if(cell_is_obstacle(x, y))
                return false;
            if(x < x2 - extraCheck)
            {
                num = (gap + error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(cell_is_obstacle(x, y + k*step_y))
                        return false;
            }
            if(x > x1 + extraCheck)
            {
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(cell_is_obstacle(x, y - k*step_y))
                        return false;
            }
            error += delta_y;
            if((error<<1) > delta_x)
            {
                y += step_y;
                error -= delta_x;
            }
        }
    }
    else
    {
        int extraCheck = int(agent_size*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON);
        for(int n = 1; n <= extraCheck; n++)
        {
            error += delta_x;
            num = (gap - error)/delta_y;
            for(k = 1; k <= num; k++)
                if(cell_is_obstacle(x1 + k*step_x, y1 - n*step_y))
                    return false;
            for(k = 1; k <= num; k++)
                if(cell_is_obstacle(x2 - k*step_x, y2 + n*step_y))
                    return false;
        }
        error = 0;
        for(y = y1; y != y2 + step_y; y += step_y)
        {
            if(cell_is_obstacle(x, y))
                return false;
            if(y < y2 - extraCheck)
            {
                num = (gap + error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(cell_is_obstacle(x + k*step_x, y))
                        return false;
            }
            if(y > y1 + extraCheck)
            {
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(cell_is_obstacle(x - k*step_x, y))
                        return false;
            }
            error += delta_x;
            if((error<<1) > delta_y)
            {
                x += step_x;
                error -= delta_y;
            }
        }
    }
    return true;
}
