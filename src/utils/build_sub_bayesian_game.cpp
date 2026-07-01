#include <iostream>
#include "two_players_bayesian_game.hpp"
#include <vector>
#include <cmath>
#include "dfs.hpp"
#include <algorithm>
#include <unordered_map>

// std::vector<std::vector<int>> build_CK_blocks(TwoPlayersBayesianGame* bg, std::vector<std::vector<int>> types, int k, float eps){
//     int n1 = types[0].size();
//     int n2 = types[1].size();

//     // build jointTypesProbabilities matrix
//     auto jointTypesProbabilities = std::vector<std::vector<float>>(n1, std::vector<float>(n2, 0.0f));
//     for (int i=0; i<n1; i++){
//         for (int j=0; j<n2; j++){
//             jointTypesProbabilities[i][j] = bg->getJointTypesProba({types[0][i], types[1][j]});
//         }
//     }

//     // transform to a graph with integer weights for Gomory-Hu tree
//     int k1 = 0;
//     int min_proba = jointTypesProbabilities[0][0];
//     for (int i=0; i<n1; i++){
//         for (int j=0; j<n2; j++){
//             if (jointTypesProbabilities[i][j] > 0 && jointTypesProbabilities[i][j] < min_proba){
//                 min_proba = jointTypesProbabilities[i][j];
//             }
//         }
//     }

//     while(min_proba*std::pow(10,k1)<1){  
//         k1++;
//     }

//     std::vector<std::vector<int>> graph = std::vector<std::vector<int>>(n1+n2, std::vector<int>(n1+n2, 0));
//     for (int i=0; i<n1; i++){
//         for (int j=0; j<n2; j++){
//             graph[i][j] = static_cast<int>(jointTypesProbabilities[i][j]*std::pow(10,k));
//         }
//     }

//     // k-cut problem
//     // build GH tree
//     GomoryHuTree gh(n1+n2, graph);

//     // remove k-1 edges + compare w eps
//     for (int i=0;i<n1+n2;i++){
//         for (int j=0;j<n1+n2;j++){
//             if (gh.graph[i][j] > 0 && gh.graph[i][j] < eps*std::pow(10,k)){
//                 gh.graph[i][j] = 0;
//             }
//         }
//     }

    

//     return graph; // tam
// }

TwoPlayersBayesianGame build_sub_bayesian_game(TwoPlayersBayesianGame* bg, const std::vector<int>& block, std::vector<std::vector<int>> types){
    TwoPlayersBayesianGame subgame;
    std::vector<std::vector<int>> local_types(types.size());
    
    // create a map from type to index 
    std::unordered_map<int, int> type_to_index;
    for (int i = 0; i < types.size(); i++) {
        for (int t : types[i]) {
            type_to_index[t] = i;
        }
    }
    
    // fill local_types with the types in the "block"
    // for (int b:block){
    //     int player = type_to_index[b];
    //     if (player != -1) {
    //         local_types[player].push_back(b);
    //     }
    // }
    for (int b : block) {
        auto it = type_to_index.find(b);
        if (it != type_to_index.end()) {
            int player = it->second;
            local_types[player].push_back(b);
        }
    }

    int n1 = local_types[0].size();
    int n2 = local_types[1].size();

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
            float p = bg->getJointTypesProba({local_types[0][i], local_types[1][j]});
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
                    std::vector<int> types = {local_types[0][i], local_types[1][j]};
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


std::vector<TwoPlayersBayesianGame> decompose_game(TwoPlayersBayesianGame* bg, block occ, std::vector<std::vector<int>> types){
    auto raw_blocks = getListBlocks(occ);

    std::vector<TwoPlayersBayesianGame> subgames;

    for (const auto &b : raw_blocks)
    {
        subgames.push_back(build_sub_bayesian_game(bg, b, types));
    }

    return subgames;
}


int main() {
    TwoPlayersBayesianGame bg;

    std::vector<int> block = {0, 2};

    std::vector<std::vector<int>> types = {{0, 1}, {2, 3}};
    
    TwoPlayersBayesianGame subgame;

    int n1 = types[0].size();
    int n2 = types[1].size();
    int A1 = 2;
    int A2 = 2;

    bg.setTypeNumbers({std::to_string(n1), std::to_string(n2)});

    bg.addJointTypeProbabilities({"0.1", "0.2", "0.3", "0.4"});

    bg.setGameDimensions({std::to_string(A1), std::to_string(A2)}); //actions

    for (int i =0; i<pow(A1,n1)*pow(A2,n2); i++){
        std::vector<std::string> line;
        int num = rand() % 10 + 1;
        line.push_back(std::to_string(num));
        bg.addPayoffLine(line);
    }

    subgame = build_sub_bayesian_game(&bg, block, types);
    return 0;
}
