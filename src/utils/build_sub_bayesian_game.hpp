#pragma once

#include <vector>
#include <utility>
#include "two_players_bayesian_game.hpp"
#include "GomoryHu.hpp"
#include "dfs.hpp"

struct MatrixWithIndices {
    std::pair<int, int> indices; 
    int weight;
};

struct SubgameMapping {
    std::vector<int> local_to_global_P1;
    std::vector<int> local_to_global_P2;
};



std::vector<MatrixWithIndices> sorted_matrix(std::vector<std::vector<int>> matrix);

bool isImportantEdge(int a, int b, const std::vector<std::pair<int,int>>& important_edges);

std::vector<std::vector<int>> build_CK_blocks(TwoPlayersBayesianGame* bg, std::vector<std::vector<int>> types, int k, float eps);

std::pair<SubgameMapping,TwoPlayersBayesianGame> build_sub_bayesian_game(TwoPlayersBayesianGame* bg, const std::vector<std::vector<int>>& block);

std::vector<std::pair<SubgameMapping,TwoPlayersBayesianGame>> decompose_game(TwoPlayersBayesianGame* bg, std::vector<std::vector<int>> CKblocks, std::vector<std::vector<int>> types);