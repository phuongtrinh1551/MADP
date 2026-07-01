#include <vector>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <unordered_set>

using support=int;
using block=std::vector<std::vector<float>>;

std::vector<std::vector<support>> getListBlocks(block occ); 

void dfs(int supportPI, int player, std::unordered_set<support> & visited,block occ, std::vector<support> & current_subgraph);

void print_blocks(std::vector<std::vector<support>> subgraphs);
