#include "Map.h"

#include <iostream>
#include <unistd.h>

Map::Map(nav_msgs::OccupancyGrid map){

    _map = map;
    uint32_t cols = map.info.width;
    uint32_t rows = map.info.height;

    _cells = new Cell**[rows];

    for(int i = 0; i<rows;i++)
    {
        _cells[i] = new Cell*[cols];

        for(int j = 0;j<cols;j++){
            geometry_msgs::Point celPos ;//= new geometry_msgs::Point;
            celPos.x = (j * map.info.resolution) + map.info.origin.position.x;
            celPos.y = (i * map.info.resolution) + map.info.origin.position.y;
            if(map.data[j+i*cols] < 80){
                if(map.data[j+i*cols]==50){
                    _cells[i][j] = new Cell(celPos,2);
                }else{
                    _cells[i][j] = new Cell(celPos);
                }

            } else{
                _cells[i][j] = new Cell(celPos, Cell::COST_UNWALKABLE);
            }
        }
    }

    //Attach neighbors

    for(uint32_t i = 0; i<rows; i++){
        for(uint32_t j = 0; j<cols;j++){
            if(_cells[i][j]!=NULL){
                Cell** nbrs = new Cell*[Cell::NUM_NBRS];

                for(unsigned int k = 0;k<Cell::NUM_NBRS;k++){
                    nbrs[k] = NULL;
                }


                //top
                if(i < rows - 1){

                    //top-left
                    if(j!=0){
                        nbrs[0] = _cells[i+1][j-1];
                    }
                    //top-middle
                    nbrs[1] = _cells[i+1][j];

                    //top-right
                    if(j<cols-1){
                        nbrs[2] = _cells[i+1][j+1];
                    }

                }

                //middle
                //middle-left
                if(j !=0){
                    nbrs[3] = _cells[i][j-1];
                }
                //middle-right
                if(j < cols - 1){
                    nbrs[4] = _cells[i][j+1];
                }

                //bottom
                if(i != 0){

                    //bottom-left
                    if(j!=0){
                        nbrs[5]= _cells[i-1][j-1];
                    }

                    //bottom-middle
                    nbrs[6] = _cells[i-1][j];

                    //bottom-right
                    if(j<cols-1){
                        nbrs[7] = _cells[i-1][j+1];
                    }
                }
                _cells[i][j]->init(nbrs);


            }



        }
    }

}

uint32_t Map::cols() {


    return _map.info.width;
}

uint32_t Map::rows() {
    return _map.info.height;
}

Cell* Map::operator()(const unsigned int row, const unsigned int col)
{
    return _cells[row][col];
}

bool Map::has(unsigned int row, unsigned int col)
{
    bool result = false;
    if(row >= 0 && row < rows() && col >= 0 && col < cols()){
        if(_cells[row][col] != NULL){
            result = true;
        }
    }
    return result;
}

Cell* Map::getCell(geometry_msgs::Point point){

    int row = (int)round((point.y - _map.info.origin.position.y) / _map.info.resolution);
    int col = (int)round((point.x - _map.info.origin.position.x) / _map.info.resolution);
    return _cells[row][col];
}

Cell* Map::getcloseWalkable(Cell* cell){
    for(int i = 0; i<Cell::NUM_NBRS;i++){
        Cell *nbr;
        nbr = cell->nbrs()[i];
        if(nbr!=NULL && nbr->cost<Cell::COST_UNWALKABLE){
            return nbr;
        }
    }
    return cell;
}