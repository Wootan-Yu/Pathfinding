#include "SFML/Graphics.hpp"
#include <vector>
#include <iostream>
#include <cmath>
#include <chrono>
#include <queue>
#include <array>
#include <map>

//steps to create grid:
//1. basics of 2d array in vectors
//2. store rect object/sprite on 2d array
//3. print rect object using the stored object in 2d array

//4. add mouse pressed function to change the color of chosen cells
//prob: if mouse is moved while pressed, it doesn't register to change color

//5. sol: line function (y = mx + b)
// b = start pos of mouse
//
// horizontal_distance(h_dist)   -   abs(x2 - x1)
// vertical_distance(v_dist)     -   abs(y2 - y1)
//
// m (slope) = v_dist / h_dist


// if( h_dist > v_dist ) //horizontal is longer
//      -every time x moves by 1, move y using line formula (y = mx + b)
//      -change cells to 'selected'

// if( h_dist < v_dist ) //vertical is longer
//      -use line formula (x = y / m), to find x in terms of y



// steps to get started with pathfinding:
//6. add start and finish (usage: left mouse button then press s - start and f - finish)
//7. 1st pathfinding algo: bfs using queues
// queues - is a ds open on both sides (First in, First Out) FIFO
//  -to add: .push(element)
//  -to remove: .pop()

// usage of queue in bfs:
// 1.at the start, queue consist of '1 cell' - start cell
// 2.then mark that cell 'visited' to avoid checking again
// 3.each step of search, remove the '1st cell' out of queue
// 4.if the cell 'finish cell', stop search.
// 5.if not get the adjacent cells(top,left,right,bottom) then 
//   include the unvisited cells in queue then mark them visited
// 6.then remove the next cell out of queue
// 7.repeat until it reaches 'finish cell' or queue becomes 'empty'


//global variables
char origin_x = 8;
char origin_y = 8;
char cell_size = 15;
unsigned char resize = 1;

char row = 43; //43
char col = 78; //78


enum Cell
{
    Empty,
    Finish,
    Start,
    Wall,
    Visited,
    Path,
    Invalid
};


//for pairing x and y values, to store it as 1
template <typename val_type = unsigned short>
using Position = std::pair<val_type, val_type>;

std::vector<Position<short>> adjacent_cells;
std::map<std::pair<char, char>, std::pair<char, char>> prev_cell; //[[parent_cell.x, parent_cell.y : child_cell.x, child_cell.y]...]


static Position<short> get_mouseCell(sf::RenderWindow& window)
{
    float mouse_x = sf::Mouse::getPosition(window).x - origin_x * resize;
    float mouse_y = sf::Mouse::getPosition(window).y - origin_y * resize;
    return Position<short>(std::floor(mouse_x / cell_size / resize), std::floor(mouse_y / cell_size / resize));
}


//for optimization, because char is used as storage instead of int or short
template <typename val_type>
static char sign(const val_type i_value)
{
    return (0 < i_value) - (0 > i_value);
}

sf::Vector2<char> finish_pos(col - 1, row - 1);
sf::Vector2<char> start_pos(0, 0);


static char get_cell(const Position<>& i_cell, const std::vector<std::vector<char>>& map)
{
    if (0 <= i_cell.first && 0 <= i_cell.second && i_cell.first < col && i_cell.second < row)
    {
        return map[i_cell.first][i_cell.second];
    }

    return Cell::Invalid;
}

static std::vector<Position<short>> get_adjacent_cell(const Position<char>& i_cell, const std::vector<std::vector<char>> map)
{
    std::array<bool, 9> valid_adjacent_cells = {}; //'valid adjacent cells' - consist of 9 cells to move (top-left, top, top-right, right, mid, left, bottom-left, bottom, bottom-right)

    std::vector<Position<short>> adjacent_cells;
    std::vector<Position<short>> diagonal_adjacent_cells;

    for (unsigned char a = 0; a < valid_adjacent_cells.size(); a++)
    {
        char cell = get_cell(Position<short>(i_cell.first + a % 3 - 1, i_cell.second + floor(a / 3.f) - 1), map);

        valid_adjacent_cells[a] = Cell::Invalid != cell && Cell::Wall != cell; //[true, false]
    }

    for (unsigned short a = 0; a < 3; a++)
    {
        for (unsigned short b = 0; b < 3; b++)
        {
            if ((1 != a || 1 != b) && 1 == valid_adjacent_cells[b + 3 * a])
            {
                if (std::abs(a - 1) == std::abs(b - 1))
                {
                    if (1 == valid_adjacent_cells[1 + 3 * a] && 1 == valid_adjacent_cells[3 + b])
                    {
                        diagonal_adjacent_cells.push_back(Position<short>(b + i_cell.first - 1, a + i_cell.second - 1));
                    }
                }
                else
                {
                    adjacent_cells.push_back(Position<short>(b + i_cell.first - 1, a + i_cell.second - 1));
                }
            }
        }
    }

    //We're adding diagonal cells at the end of the array so that the BFS doesn't make unnecessary turns.
    //P. S. I wrote "unnecessary" correctly on the first try!
    //adjacent_cells.insert(adjacent_cells.end(), diagonal_adjacent_cells.begin(), diagonal_adjacent_cells.end());

    return adjacent_cells;
}

static void bfs_manual(const Position<short>& finish_pos, const Position<short>& start_pos, std::vector<std::vector<char>>& map)
{
    std::queue<Position<short>> path_queue; // [[x,y], [x,y],....]
    path_queue.push(start_pos); //[[startPos_x, startPos_y]]
    
    //std::map<std::pair<char, char>, std::pair<char, char>> prev_cell; //[[parent_cell.x, parent_cell.y : child_cell.x, child_cell.y]...]

    map[start_pos.first][start_pos.second] = Cell::Start;

    //std::cout << "x:" << path_queue.front().first << " y:" << path_queue.front().second << '\n';


    while (0 == path_queue.empty())
    {
        std::vector<Position<short>> adjacent_cells; //[[x, y], [x, y], ....]
        Position<short> cell = path_queue.front();

        adjacent_cells = get_adjacent_cell(cell, map);

        path_queue.pop();

        if (cell == finish_pos)
        {
            return;
        }

        for (const Position<short>& adjacent_cell : adjacent_cells)
        {
            if (Cell::Visited != map[adjacent_cell.first][adjacent_cell.second])
            {
                map[adjacent_cell.first][adjacent_cell.second] = Cell::Visited; //visited
                path_queue.push(adjacent_cell);
            }
        }
    }
}

static void bfs_auto(const Position<short>& finish_pos, const Position<short>& start_pos, std::vector<std::vector<char>>& map)
{
    std::queue<sf::Vector2<char>> path_queue;

    path_queue.push(sf::Vector2<char>(start_pos.first, start_pos.second));

    while (0 == path_queue.empty())
    {
        sf::Vector2<char> cell = path_queue.front();

        adjacent_cells = get_adjacent_cell(Position<short>(cell.x, cell.y), map);

        path_queue.pop();

        if (Position<short>(cell.x, cell.y) == finish_pos)
        {
            std::pair<char, char> path_cell = std::pair<char, char>(cell.x, cell.y);

            do
            {
                map[path_cell.first][path_cell.second] = Cell::Path;

                path_cell = prev_cell.at(path_cell);

            } while (start_pos.first != path_cell.first || start_pos.second != path_cell.second);

            return;
        }

        for (const Position<short>& adjacent_cell : adjacent_cells)
        {
            if (Cell::Visited != map[adjacent_cell.first][adjacent_cell.second])
            {
                map[adjacent_cell.first][adjacent_cell.second] = Cell::Visited; //visited

                prev_cell[std::pair<char, char>(adjacent_cell.first, adjacent_cell.second)] = std::pair<char, char>(cell.x, cell.y);

                path_queue.push(sf::Vector2<char>(adjacent_cell.first, adjacent_cell.second));
            }
        }
    }
    
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(1280, 720), "Window Title");

    window.setFramerateLimit(60);

    sf::RectangleShape rect(sf::Vector2f(cell_size, cell_size));
    rect.setFillColor(sf::Color(36, 36, 85));
    rect.setOutlineColor(sf::Color::Black);
    rect.setOutlineThickness(2.0f);


    std::vector<std::vector<char>> map_nums; //storage numbers/enums
    std::vector<std::vector<sf::RectangleShape>> map_rect; //storage rectangle


    for (size_t i = 0; i < col; i++)
    {
        std::vector<char> map_num_col;
        std::vector<sf::RectangleShape> map_rect_col;

        for (size_t j = 0; j < row; j++)
        {
            map_num_col.push_back(0);
            map_rect_col.push_back(rect);
        }

        map_nums.push_back(map_num_col);
        map_rect.push_back(map_rect_col);
    }

    /*std::cout << "row: " << map_rect[0].size() << '\n';
    std::cout << "col: " << map_rect.size() << '\n';*/



    //time
    //This will make the program run at 60 FPS.
    //Because 1 second == 1,000,000 microseconds
    //1,000,000 microseconds / 60 frames = 16667 microseconds per frame

    std::chrono::microseconds FRAME_DURATION(16667);
    std::chrono::microseconds lag(0);
    std::chrono::steady_clock::time_point previous_time;
    previous_time = std::chrono::steady_clock::now();

    char mouse_pressed = 0;
    Position<short> mouse_cell_start; //used to draw lines of cells. line eq: y = mx + b

    //for (size_t a = 0; a < map_nums.size(); a++) //row
    //{
    //    for (size_t b = 0; b < map_nums[0].size(); b++) //col
    //    {
    //        std::cout << map_nums[a][b] << " ";
    //    }
    //    std::cout << '\n';
    //}

    sf::Event event;
    while (window.isOpen())
    {
        std::chrono::microseconds delta_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - previous_time);

        lag += delta_time;
        previous_time += delta_time;

        while (FRAME_DURATION <= lag)
        {
            bool map_updated = 0;
            lag -= FRAME_DURATION;

            while (window.pollEvent(event))
            {
                if (event.type == sf::Event::Closed)
                    window.close();

                if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left) || sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
                {
                    if (0 == mouse_pressed)
                    {
                        mouse_pressed = 1;

                        mouse_cell_start = get_mouseCell(window);

                        //float mouse_x = sf::Mouse::getPosition(window).x - origin_x * resize;
                        //float mouse_y = sf::Mouse::getPosition(window).y - origin_y * resize;

                        //if (0 <= mouse_x && 0 <= mouse_y &&
                        //    mouse_x < cell_size * col * resize && mouse_y < cell_size * col * resize) //check if mouse is inside the cells
                        //{
                        //    unsigned int cell_x = std::floor(mouse_x / cell_size / resize);
                        //    unsigned int cell_y = std::floor(mouse_y / cell_size / resize);

                        //    std::cout << "x: " << cell_x << " y:" << cell_y << '\n';

                        //    if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
                        //    {
                        //        v[cell_x][cell_y].setFillColor(sf::Color::White);
                        //    }

                        //    if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
                        //    {
                        //        v[cell_x][cell_y].setFillColor(sf::Color::Red);
                        //    }
                        //}
                    }
                }
                else
                {
                    mouse_pressed = 0;
                }

                if (1 == mouse_pressed)
                {
                    //b - start of mouse pos
                    Position<short> mouse_cell = get_mouseCell(window);

                    // horizontal_distance(h_dist)   -   abs(x2 - x1)
                    // vertical_distance(v_dist)     -   abs(y2 - y1)

                    // m (slope) = v_dist / h_dist
                    unsigned short line_length = 1 + std::max(std::abs(mouse_cell.first - mouse_cell_start.first), std::abs(mouse_cell.second - mouse_cell_start.second));

                    // x and y value in y = mx + b
                    char step_x = sign(mouse_cell.first - mouse_cell_start.first);
                    char step_y = sign(mouse_cell.second - mouse_cell_start.second);

                    for (size_t a = 0; a < line_length; a++)
                    {
                        Position<short> cell;

                        //We take 1 step in one direction and use the slope to calculate the step in the other direction.
                        //line formula: y = mx + b
                        cell.first = mouse_cell_start.first + step_x * floor(a * (1.f + std::abs(mouse_cell.first - mouse_cell_start.first)) / static_cast<float>(line_length));
                        cell.second = mouse_cell_start.second + step_y * floor(a * (1.f + std::abs(mouse_cell.second - mouse_cell_start.second)) / static_cast<float>(line_length));

                        if (0 <= cell.first && 0 <= cell.second &&
                            cell.first < col && cell.second < row) //check if mouse is inside the map of cells
                        {
                            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
                            {
                                map_updated = 1;
                                if (sf::Keyboard::isKeyPressed(sf::Keyboard::F))
                                {
                                    //map_rect[finish_pos.x][finish_pos.y].setFillColor(sf::Color(36, 36, 85)); //empty
                                    //map_rect[cell.first][cell.second].setFillColor(sf::Color::Red); //finish
                                    map_nums[finish_pos.x][finish_pos.y] = Cell::Empty;
                                    map_nums[cell.first][cell.second] = Cell::Finish;

                                    finish_pos = sf::Vector2<char>(cell.first, cell.second);
                                    //std::cout << "finish_pos - x: " << finish_pos.x << " y: " << finish_pos.y << '\n';
                                }
                                else if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
                                {
                                    //map_rect[start_pos.x][start_pos.y].setFillColor(sf::Color(36, 36, 85)); //empty
                                    //map_rect[cell.first][cell.second].setFillColor(sf::Color::Green); //start
                                    map_nums[start_pos.x][start_pos.y] = Cell::Empty;
                                    map_nums[cell.first][cell.second] = Cell::Start;

                                    start_pos = sf::Vector2<char>(cell.first, cell.second);
                                    //std::cout << "start_pos - x: " << start_pos.x << " y: " << start_pos.y << '\n';
                                }
                                //else if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
                                //{
                                //    map_rect[cell.first][cell.second].setFillColor(sf::Color::White); //wall
                                //    map_nums[cell.first][cell.second] = Cell::Wall;
                                //}
                            }

                            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
                            {
                                /*map_updated = 1;
                                map_nums[start_pos.x][start_pos.y] = Cell::Empty;
                                map_nums[cell.first][cell.second] = Cell::Start;

                                start_pos = sf::Vector2<char>(cell.first, cell.second);*/

                                map_rect[cell.first][cell.second].setFillColor(sf::Color::White); //wall
                                map_nums[cell.first][cell.second] = Cell::Wall;
                            }
                        }

                    }

                    mouse_cell_start = mouse_cell;

                    
                }
            }
            if (1 == map_updated)
            {
                for (auto& column : map_nums)
                {
                    for (auto& cell : column)
                    {
                        if (Cell::Visited == cell) //visited = 4
                        {
                            cell = Cell::Empty; //empty = 0
                        }
                    }
                }

                bfs_auto(Position<short>(finish_pos.x, finish_pos.y), Position<short>(start_pos.x, start_pos.y), map_nums);
            }

            if (FRAME_DURATION > lag)
            {
                window.clear(sf::Color::Black); // Color background

                for (size_t a = 0; a < map_nums.size(); a++) //row
                {
                    for (size_t b = 0; b < map_nums[0].size(); b++) //col
                    {
                        map_rect[a][b].setPosition(origin_x + a * cell_size, origin_y + b * cell_size);


                        if (a == finish_pos.x && b == finish_pos.y)
                        {
                            map_rect[a][b].setFillColor(sf::Color::Red);
                        }
                        else if (a == start_pos.x && b == start_pos.y)
                        {
                            map_rect[a][b].setFillColor(sf::Color::Green);
                        }
                        else
                        {
                            //create a switch statement that chooses the type of cell
                            //empty: sf::Color(36, 36, 85)
                            //path:
                            //visited:
                            //wall:

                            switch (map_nums[a][b])
                            {
                                case Cell::Empty:
                                    map_rect[a][b].setFillColor(sf::Color(36, 36, 85));
                                    break;

                                /*case Cell::Finish:
                                    map_rect[a][b].setFillColor(sf::Color::Red);
                                    break;

                                case Cell::Start:
                                    map_rect[a][b].setFillColor(sf::Color::Green);
                                    break;*/

                                case Cell::Wall:
                                    map_rect[a][b].setFillColor(sf::Color::White);
                                    break;

                                case Cell::Path:
                                    map_rect[a][b].setFillColor(sf::Color::Yellow);
                                    break;

                                case Cell::Visited:
                                    map_rect[a][b].setFillColor(sf::Color::Blue);
                                    break;
                            }
                        }

                        

                        window.draw(map_rect[a][b]);
                    }
                }

                window.display();
            }

        }
    }
    return 0;
}