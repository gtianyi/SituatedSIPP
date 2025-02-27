#include "map.h"
using namespace tinyxml2;

Map::Map()
{
    height = 0;
    width = 0;
}
Map::~Map()
{
    Grid.clear();
}

bool Map::getMap(const char* FileName)
{
    XMLDocument doc;
    if(doc.LoadFile(FileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning map input XML file."<<std::endl;
        return false;
    }

    XMLElement *root = nullptr;
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML map-file."<<std::endl;
        return false;
    }

    XMLElement *map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "No '"<<CNS_TAG_MAP<<"' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *grid = map->FirstChildElement(CNS_TAG_GRID);
    if (!grid)
    {
        std::cout << "No '"<<CNS_TAG_GRID<<"' element found in XML file."<<std::endl;
        return false;
    }
    height = grid->IntAttribute(CNS_TAG_ATTR_HEIGHT);
    if(height <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_HEIGHT<<" attribute. It should be >0.\n";
        return false;
    }
    width = grid->IntAttribute(CNS_TAG_ATTR_WIDTH);
    if(width <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_WIDTH<<" attribute. It should be >0.\n";
        return false;
    }
    XMLElement *row = grid->FirstChildElement(CNS_TAG_ROW);
    Grid.resize(height);
    for(int i = 0; i < height; i++)
        Grid[i].resize(width, 0);

    std::string value;
    const char* rowtext;
    std::stringstream stream;
    for(int i = 0; i < height; i++)
    {
        if (!row)
        {
            std::cout << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given." << std::endl;
            return false;
        }

        rowtext = row->GetText();
        unsigned int k = 0;
        value = "";
        int j = 0;

        for(k = 0; k < strlen(rowtext); k++)
        {
            if (rowtext[k] == ' ')
            {
                stream << value;
                stream >> Grid[i][j];
                stream.clear();
                stream.str("");
                value = "";
                j++;
            }
            else
            {
                value += rowtext[k];
            }
        }
        stream << value;
        stream >> Grid[i][j];
        stream.clear();
        stream.str("");

        if (j < width-1)
        {
            std::cout << "Not enough cells in '" << CNS_TAG_ROW << "' " << i << " given." << std::endl;
            return false;
        }
        row = row->NextSiblingElement(CNS_TAG_ROW);
    }
    return true;
}


bool Map::CellIsTraversable(int i, int j) const
{
    return (Grid[i][j] == 0);
}

bool Map::CellIsObstacle(int i, int j) const
{
    return (Grid[i][j] != 0);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

int Map::getValue(int i, int j) const
{
    if(i < 0 || i >= height)
        return -1;
    if(j < 0 || j >= width)
        return -1;

    return Grid[i][j];
}

std::vector<Node> Map::getValidMoves(int i, int j, int k, double size) const
{
   LineOfSight los;
   los.setSize(size);
   std::vector<Node> moves;
   if(k == 2)
       moves = {Node(0,1,0,1.0),   Node(1,0,1,1.0),         Node(-1,0,2,1.0), Node(0,-1,3,1.0)};
   else if(k == 3)
       moves = {Node(0,1,0,1.0),   Node(1,1,1,sqrt(2.0)),   Node(1,0,2,1.0),  Node(1,-1,3,sqrt(2.0)),
                Node(0,-1,4,1.0),  Node(-1,-1,5,sqrt(2.0)), Node(-1,0,6,1.0), Node(-1,1,7,sqrt(2.0))};
   else if(k == 4)
       moves = {Node(0,1,0,1.0),          Node(1,1,1,sqrt(2.0)),    Node(1,0,2,1.0),          Node(1,-1,3,sqrt(2.0)),
                Node(0,-1,4,1.0),         Node(-1,-1,5,sqrt(2.0)),  Node(-1,0,6,1.0),         Node(-1,1,7,sqrt(2.0)),
                Node(1,2,8,sqrt(5.0)),    Node(2,1,9,sqrt(5.0)),    Node(2,-1,10,sqrt(5.0)),   Node(1,-2,11,sqrt(5.0)),
                Node(-1,-2,12,sqrt(5.0)),  Node(-2,-1,13,sqrt(5.0)),  Node(-2,1,14,sqrt(5.0)),   Node(-1,2,15,sqrt(5.0))};
   else
       moves = {Node(0,1,0,1.0),          Node(1,1,1,sqrt(2.0)),    Node(1,0,2,1.0),          Node(1,-1,3,sqrt(2.0)),
                Node(0,-1,4,1.0),         Node(-1,-1,5,sqrt(2.0)),  Node(-1,0,6,1.0),         Node(-1,1,7,sqrt(2.0)),
                Node(1,2,8,sqrt(5.0)),    Node(2,1,9,sqrt(5.0)),    Node(2,-1,10,sqrt(5.0)),   Node(1,-2,11,sqrt(5.0)),
                Node(-1,-2,12,sqrt(5.0)),  Node(-2,-1,13,sqrt(5.0)),  Node(-2,1,14,sqrt(5.0)),   Node(-1,2,15,sqrt(5.0)),
                Node(1,3,16,sqrt(10.0)),   Node(2,3,17,sqrt(13.0)),   Node(3,2,18,sqrt(13.0)),   Node(3,1,19,sqrt(10.0)),
                Node(3,-1,20,sqrt(10.0)),  Node(3,-2,21,sqrt(13.0)),  Node(2,-3,22,sqrt(13.0)),  Node(1,-3,23,sqrt(10.0)),
                Node(-1,-3,24,sqrt(10.0)), Node(-2,-3,25,sqrt(13.0)), Node(-3,-2,26,sqrt(13.0)), Node(-3,-1,27,sqrt(10.0)),
                Node(-3,1,28,sqrt(10.0)),  Node(-3,2,29,sqrt(13.0)),  Node(-2,3,30,sqrt(13.0)),  Node(-1,3,31,sqrt(10.0))
                //Node(2,0,2,2.0),           Node(-2,0,6,2.0),          Node(0,-2,4,2.0),          Node(0,2,0,2.0),
                //Node(2,2,1,2.0*sqrt(2.0)), Node(-2,2,7,2.0*sqrt(2.0)),Node(2,-2,3,2.0*sqrt(2.0)),Node(-2,-2,5,2.0*sqrt(2.0)),
                //Node(0,3,0,3.0),           Node(-3,0,6,3.0),          Node(0,-3,4,3.0),          Node(3,0,2,3.0),
                //Node(3,3,1,3.0*sqrt(2.0)), Node(-3,3,7,3.0*sqrt(2.0)),Node(3,-3,3,3.0*sqrt(2.0)),Node(-3,-3,5,3.0*sqrt(2.0))
                };
   std::vector<bool> valid(moves.size(), true);
   for(int k = 0; k < moves.size(); k++)
       if(!CellOnGrid(i + moves[k].i, j + moves[k].j) || CellIsObstacle(i + moves[k].i, j + moves[k].j)
               || !los.checkLine(i, j, i + moves[k].i, j + moves[k].j, *this))
           valid[k] = false;
   std::vector<Node> v_moves = {};
   for(int k = 0; k < valid.size(); k++)
       if(valid[k])
           v_moves.push_back(moves[k]);
   return v_moves;
}

std::vector<RTNode> Map::getValidRTMoves(int i, int j, int k, double size) const{
   std::vector<Node> n_node_moves = getValidMoves(i, j, k, size);
   std::vector<RTNode> v_moves = {};
   for(const Node& n: n_node_moves){
       v_moves.emplace_back(n.i, n.j, n.g, 0.0, n.heading_id);
   }
   return v_moves;
}
