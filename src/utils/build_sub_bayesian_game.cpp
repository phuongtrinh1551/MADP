#include <iostream>
#include "two_players_bayesian_game.hpp"
#include <vector>
#include <cmath>
#include "dfs.hpp"
#include <algorithm>
#include <unordered_map>
#include "GomoryHu.hpp"

struct MatrixWithIndices {
    std::pair<int, int> indices; 
    int weight;
};

std::vector<MatrixWithIndices> sorted_matrix(std::vector<std::vector<int>> matrix) {
    int rows = matrix.size();
    int cols = matrix[0].size();

    std::vector<MatrixWithIndices> flat_matrix;

    for(int i=0; i<rows; i++) {
        for(int j=0; j<cols; j++) {
            flat_matrix.push_back({{i, j}, matrix[i][j]});
        }
    }

    std::sort(flat_matrix.begin(), flat_matrix.end(), [](const MatrixWithIndices &a, const MatrixWithIndices &b) {
        return a.weight < b.weight;
    });

    return flat_matrix; 
}


std::vector<std::vector<int>> build_CK_blocks(TwoPlayersBayesianGame* bg, std::vector<std::vector<int>> types, int k, float eps){
    // BUILD GRAPH OF CK BLOCKS
    int n1 = types[0].size();
    int n2 = types[1].size();

    // transform to a graph with integer weights for Gomory-Hu tree
    int c = 0;
    int min_proba = bg->getJointTypesProba({0,0});
    for (int i=0; i<n1; i++){
        for (int j=0; j<n2; j++){
            if (bg->getJointTypesProba({i,j}) > 0 && bg->getJointTypesProba({i,j}) < min_proba){
                min_proba = bg->getJointTypesProba({i,j});
            }
        }
    }
    
    // find the smallest c st min_proba*10^c >= 1
    while(min_proba*std::pow(10,c)<1){  
        c++;
    }

    std::vector<std::vector<int>> graph = std::vector<std::vector<int>>(n1+n2, std::vector<int>(n1+n2, 0));
    for (int i = 0; i < n1; i++) {
        for (int j = 0; j < n2; j++) {
            graph[i][j+n1] = static_cast<int>(bg->getJointTypesProba({i, j}) * std::pow(10, c));
            graph[j+n1][i] = static_cast<int>(bg->getJointTypesProba({i, j}) * std::pow(10, c));
        }
    }

    // k-cut problem: build GH tree -> sort edges -> remove edges with weight < eps*10^c
    // build GH tree
    GomoryHuTree ght(n1+n2, graph);
    std::vector<std::vector<int>> gomory_hu_tree = ght.buildGomoryHuTree();

    // sort edges by weight with gomoryhu tree (=> same vertices as the original graph)
    std::vector<MatrixWithIndices> sorted_edges = sorted_matrix(gomory_hu_tree);
    
    // try to remove maximum edges + respect eps*10^c
    int sum_removed_edges = 0;
    int removed_edges = 0;
    for (int i=0;i<sorted_edges.size();i++){
        if (sorted_edges[i].weight > 0 && sorted_edges[i].weight < eps*std::pow(10,c)){
            graph[sorted_edges[i].indices.first][sorted_edges[i].indices.second] = 0;
            graph[sorted_edges[i].indices.second][sorted_edges[i].indices.first] = 0;
            sum_removed_edges += sorted_edges[i].weight;
            removed_edges++;
        }
    }

    // reverse transform to a matrix proba
    std::vector<std::vector<float>> matrix_proba(n1, std::vector<float>(n2, 0.0f));
    for (int i = 0; i < n1; i++) {
        for (int j = 0; j < n2; j++) {
            matrix_proba[i][j] = static_cast<float>(graph[i][j+n1]) / (std::pow(10,c)); 
        }
    }

    // USE dfs TO FIND THE CONNECTED COMPONENTS OF THE GRAPH
    std::vector<std::vector<int>> CKblocks = getListBlocks(matrix_proba);

    return CKblocks;
}


TwoPlayersBayesianGame build_sub_bayesian_game(TwoPlayersBayesianGame* bg, const std::vector<std::vector<int>>& block, std::vector<std::vector<int>> types){
    TwoPlayersBayesianGame subgame;

    int n1 = block[0].size();
    int n2 = block[1].size();

    subgame.typesNumbers = {n1, n2};
    subgame.gameDimensions = bg->getGameDimensions();

    int A1 = subgame.gameDimensions[0];
    int A2 = subgame.gameDimensions[1];

    // joint proba
    subgame.jointTypeProbabilities =
        std::vector<std::vector<float>>(n1, std::vector<float>(n2, 0.0f));

    float totalProb = 0.0f;

    for (int i = 0; i < n1; i++){
        for (int j = 0; j < n2; j++){
            float p = bg->getJointTypesProba({block[0][i], block[1][j]});
            subgame.jointTypeProbabilities[i][j] = p;
            totalProb += p;
        }
    }
    
    // normalize
    if (totalProb > 0.0f){
        for (int i = 0; i < n1; i++){
            for (int j = 0; j < n2; j++){
                subgame.jointTypeProbabilities[i][j] /= totalProb;
            }
        }      
    }

    // payoff
    subgame.payoffMatrixes = std::vector<std::vector<float>>(n1 * A1, std::vector<float>(n2 * A2, 0.0f));

    for (int i = 0; i < n1; i++){
        for (int j = 0; j < n2; j++){
            for (int a1 = 0; a1 < A1; a1++){
                for (int a2 = 0; a2 < A2; a2++){
                    std::vector<int> types = {block[0][i], block[1][j]};
                    std::vector<int> actions = {a1, a2};

                    float v = bg->getPayoff(types, actions, 0);

                    int row = i * A1 + a1;
                    int col = j * A2 + a2;

                    subgame.payoffMatrixes[row][col] = v;
                }
            }
        }
    }

    return subgame;
}


std::vector<TwoPlayersBayesianGame> decompose_game(TwoPlayersBayesianGame* bg, std::vector<std::vector<int>> types){
    std::vector<std::vector<int>> CKblocks = build_CK_blocks(bg,types, 1, 0.01);

    std::vector<TwoPlayersBayesianGame> subgames;

    for (int i = 0; i < CKblocks.size(); i++){
        std::vector<std::vector<int>> block = {CKblocks[i], CKblocks[i]};
        TwoPlayersBayesianGame subgame = build_sub_bayesian_game(bg, block, types);
        subgames.push_back(subgame);
    }

    return subgames;
}


int main() {
    TwoPlayersBayesianGame bg;

    std::vector<std::vector<int>> block = {{0,1}, {0,1}};

    std::vector<std::vector<int>> types = {{0, 1, 2, 3}, {0, 1, 2, 3}};
    
    TwoPlayersBayesianGame subgame;

    int n1 = types[0].size();
    int n2 = types[1].size();
    int A1 = 2;
    int A2 = 2;

    bg.setTypeNumbers({std::to_string(n1), std::to_string(n2)});

    // bg.addJointTypeProbabilities({"0.1", "0.2", "0.3", "0.4"}); => it creates a vector 1x4, but we want 2x2 : bg.jointTypeProbabilities = {{0.1, 0.2}, {0.3, 0.4}};
    bg.addJointTypeProbabilities({"0.05", "0.05", "0.01", "0.01"}); //type 0 of P1 with type 0 and 1 of P2
    bg.addJointTypeProbabilities({"0.2", "0.2", "0.01", "0.01"}); //type 1 of P1 with type 0 and 1 of P2
    bg.addJointTypeProbabilities({"0.02", "0.02", "0.05", "0.1"}); 
    bg.addJointTypeProbabilities({"0.02", "0.05", "0.11", "0.1"}); 

    bg.setGameDimensions({std::to_string(A1), std::to_string(A2)}); //actions

    for (int i =0; i<pow(A1,n1)*pow(A2,n2); i++){
        std::vector<std::string> line;
        int num = rand() % 10 + 1;
        line.push_back(std::to_string(num));
        bg.addPayoffLine(line);
    }

    subgame = build_sub_bayesian_game(&bg, block, types);
    // // debug
    // std::cout << "nb row bg: " << bg.jointTypeProbabilities.size() << std::endl;
    // if(bg.jointTypeProbabilities.size() > 0) {
    //     std::cout << "nb col of row 0: " << bg.jointTypeProbabilities[0].size() << std::endl;
    // }
    std::cout<< "Original game joint type probabilities: " << std::endl;
    std::cout << bg.getJointTypesProba({0,0}) << " " << bg.getJointTypesProba({1,1}) << std::endl;
    std::cout << "Subgame joint type probabilities: " << std::endl;
    std::cout << subgame.getJointTypesProba({0,0}) << std::endl;

    // debug : sorted_matrix()
    std::vector<std::vector<int>> matrix = {{3, 1, 2}, {6, 5, 4}};
    std::vector<MatrixWithIndices> sm = sorted_matrix(matrix);
    std::cout << "Sorted matrix with indices: " << std::endl;
    for (int i=0; i<sm.size(); i++){
        std::cout << "Weight: " << sm[i].weight << ", Indices: (" << sm[i].indices.first << ", " << sm[i].indices.second << ")" << std::endl;
    }

    // std::vector<std::vector<int>> vect = build_CK_blocks(&bg, types, 1, 0.05);
    // std::cout << "CK blocks: " << std::endl;
    // for (int i=0; i<vect.size(); i++){
    //     std::cout << "Block " << i << ": ";
    //     for (int j=0; j<vect[i].size(); j++){
    //         std::cout << vect[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // for (int i=0;i<vect[0].size();i++){
    //     std::cout << "vect[0][" << i << "] = " << vect[0][i] << std::endl;
    // }
    return 0;
}
