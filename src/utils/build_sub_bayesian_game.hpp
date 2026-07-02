#pragma once

#include <vector>
#include "two_players_bayesian_game.hpp"
#include "GomoryHu.hpp"
#include "dfs.hpp"

struct MatrixWithIndices {
    std::pair<int, int> indices; 
    int weight;
};
std::vector<std::vector<MatrixWithIndices>> sorted_matrix(std::vector<std::vector<int>> matrix);

std::vector<std::vector<int>> build_CK_blocks(TwoPlayersBayesianGame* bg, std::vector<std::vector<int>> types);

TwoPlayersBayesianGame build_sub_bayesian_game(TwoPlayersBayesianGame* bg, const std::vector<int>& block, int n);

std::vector<TwoPlayersBayesianGame> decompose_game(TwoPlayersBayesianGame* bg, const std::vector<std::vector<int>>& blocks, int n);