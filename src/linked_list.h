#ifndef LINKED_LIST
#define LINKED_LIST

#include <iostream>
#include <vector>

// structure for a grid
struct grid {
    int x, y;
    int weight;
    grid *next;
    grid *prev;
    int index;
};

// linked lists class implementation
class LinkedList {
    public:
        // constructor 
        // gets the first element
        LinkedList(int x, int y) {
            grid * newGrid = new grid();
            newGrid->x = x; newGrid->y = y; newGrid->index = len;
            newGrid->next = nullptr; newGrid->prev = nullptr;
            head = newGrid;
            bottom = newGrid;
            len++;
        };

        LinkedList() {

        };

        // destructor
        // deletes dynamically allocated memory
        ~LinkedList() {
            grid * currGrid = head;
            while(currGrid != nullptr) {
                grid * nextGrid = currGrid->next;
                delete currGrid;
                currGrid = nextGrid;
            }
        }

        // method for appending a new grid to the end of the list
        void append(int x, int y) {
            grid * newGrid = new grid();
            newGrid->x = x; newGrid->y = y; newGrid->index = len;
            if (len == 0) {
                newGrid->next = nullptr; newGrid->prev = nullptr;
                head = newGrid;
                bottom = newGrid;
                len++;
                return;
            }
            newGrid->next = nullptr; newGrid->prev = bottom;
            bottom->next = newGrid;
            bottom = newGrid;
            len++;
        };

        // method for appending a new grid to the head of the list
        void push(int x, int y) {
            grid * newGrid = new grid(); newGrid->index = 0;
            newGrid->x = x; newGrid->y = y;
            if (len == 0) {
                newGrid->next = nullptr; newGrid->prev = nullptr;
                head = newGrid;
                bottom = newGrid;
                len++;
                return;
            }
            grid * currGrid = head;
            while (currGrid != nullptr) {
                currGrid->index++;
                currGrid = currGrid->next;
            }
            newGrid->next = head; newGrid->prev = nullptr;
            head->prev = newGrid;
            head = newGrid;
            
            len++;
        };

        // method for removing the last element of the list
        grid * pop() {
            if (len == 0) {
                std::cerr << "List is empty. Cannot pop any element" << std::endl;
                return nullptr;
            }
            grid * beforeBottom = bottom->prev;
            grid * removedBottom = bottom;
            beforeBottom->next = nullptr;
            bottom = beforeBottom;
            removedBottom->prev = nullptr;
            len--;
            if (!adjGrids.empty()) adjGrids.pop_back();
            return removedBottom;
        };

        // method for removing the head of the list
        grid * shift() {
            if (len == 0) {
                std::cerr << "List is empty. Cannot shift any element" << std::endl;
                return nullptr;
            }
            grid * afterHead = head->next;
            grid * removedHead = head;
            afterHead->prev = nullptr;
            head = afterHead;
            removedHead->next = nullptr;
            len--;
            grid * currGrid = head;
            while (currGrid != nullptr) {
                currGrid->index--;
                currGrid = currGrid->next;
            }
            if (!adjGrids.empty()) adjGrids.erase(adjGrids.begin() + 0);
            return removedHead;
        };

        // prints out the entire list
        void print() {
            if (len == 0) {
                std::cerr << "List is empty. Cannot print any element" << std::endl;
                return;
            }
            grid * currGrid = head;
            while (currGrid != nullptr) {
                std::cout << "[" << currGrid->x << "][" << currGrid->y << "]" << std::endl;
                currGrid = currGrid->next;
            }
        };
        // poly morphism, if you print with passing an integer prints out that index
        void print(int index) {
            if (len == 0) {
                std::cerr << "List is empty. Cannot print any element" << std::endl;
                return;
            }
            if (index >= len || index < 0) {
                std::cerr << "index out of range" << std::endl;
                return;
            }
            grid * currGrid = head;
            for (int i = 0; i != index; i++) {
                currGrid = currGrid->next;
            }
            std::cout << "[" << currGrid->x << "][" << currGrid->y << "]" << std::endl;
        }

        // checking if an element exists inside the list
        bool has(int x, int y) {
            bool hasGrid = false;
            grid * currGrid = head;

            while(currGrid != nullptr) {
                if (currGrid->x == x && currGrid->y == y) {
                    hasGrid = true;
                    break;
                }
                currGrid = currGrid->next;
            }
            return hasGrid;
        };

        // returns the length of the list
        int length() {
            return len;
        };

        // finds the adjacent grids of all the current grids inside the list
        void findAdjGrids() {
            if (len == 0) {
                std::cerr << "List is empty. Cannot find adjacent grids" << std::endl;
                return;
            }
            adjGrids.clear();
            adjGrids.resize(len);
            grid * baseGrid = head;
            int baseIndex = 0;

            while (baseGrid != nullptr) {
                std::vector<grid*> &baseGridAdj = adjGrids[baseIndex];
                grid * currGrid = head;
                while (currGrid != nullptr) {
                    if (currGrid != baseGrid && 
                        ((abs(baseGrid->x - currGrid->x) == 1 && baseGrid->y == currGrid->y) || 
                         (abs(baseGrid->y - currGrid->y) == 1 && baseGrid->x == currGrid->x))) {
                            baseGridAdj.push_back(currGrid);
                    }
                    currGrid = currGrid->next;
                }
                baseGrid = baseGrid->next;
                baseIndex++;
            }

        };

        // polymorphism
        // for no input argument prints out every gird and its adjacent grids
        void printAdjGrids() {
            if (len == 0) {
                std::cerr << "List is empty. Cannot find adjacent grids" << std::endl;
                return;
            }
            grid * currGrid = head;
            for (int i = 0; i < adjGrids.size(); i++) {
                std::cout << "adjacent grids for [" << currGrid->x << "][" << currGrid->y << "]:" << std::endl;
                for (int j = 0; j < adjGrids[i].size(); j++) {
                    std::cout << "[" << adjGrids[i][j]->x << "][" << adjGrids[i][j]->y << "]" << std::endl;
                }
                currGrid = currGrid->next;
            }
        };

        // with an input argument prints out the grid and its adjacent grids at that index
        void printAdjGrids(int index) {
            if (len == 0) {
                std::cerr << "List is empty. Cannot find adjacent grids" << std::endl;
                return;
            }
            if (index >= len || index < 0) {
                std::cerr << "index out of range" << std::endl;
                return;
            }
            grid * currGrid = head;
            for (int i = 0; i != index; i++) {
                currGrid = currGrid->next;
            }
            std::cout << "adjacent grids for [" << currGrid->x << "][" << currGrid->y << "]:" << std::endl;
            for (int j = 0; j < adjGrids[index].size(); j++) {
                    std::cout << "[" << adjGrids[index][j]->x << "][" << adjGrids[index][j]->y << "]" << std::endl;
            }
        };

        // returns the index of a grid if exists
        // if the grid does not exist it returns -1
        int indexOf(int x, int y) {
            grid * currGrid = head;
            while (currGrid != nullptr) {
                if (currGrid->x - x == 0 && currGrid->y - y == 0) {
                    return currGrid->index;
                }
                currGrid = currGrid->next;
            }
            std::cerr << "Not found!!" << std::endl;
            return -1;
        };

        grid * findGrid(int x, int y) {
            grid * currGrid = head;
            while(currGrid != nullptr) {
                if (currGrid->x == x && currGrid->y == y) {
                    return currGrid;
                }
                currGrid = currGrid->next;
            }
            return nullptr;
        };

        std::vector<grid*> returnAdjGrids(int index) {
            return adjGrids[index];
        };

        void printWeights() {
            if (len == 0) {
                std::cerr << "List is empty. Cannot print any element" << std::endl;
                return;
            }
            grid * currGrid = head;
            while (currGrid != nullptr) {
                std::cout << "[" << currGrid->x << "][" << currGrid->y << "].weight: " << currGrid->weight << std::endl;
                currGrid = currGrid->next;
            }
        };

        grid* getHead() const {
            return head;
        };

    private:
        grid * head;
        grid * bottom;
        std::vector<std::vector<grid*>> adjGrids;
        int len = 0;
};
#endif