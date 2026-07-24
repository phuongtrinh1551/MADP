#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <set>

#include "Globals.h"
#include "BayesianGameIdenticalPayoff.h"
#include "BGIP_SolverBruteForceSearch.h"
#include "JointPolicyPureVector.h"
#include <build_sub_bayesian_game.hpp>
#include <two_players_bayesian_game.hpp>
#include "JointPolicy.h"
#include "JointPolicyPureVector.h"

int main () { 
    // //===== generate TwoPlayersBayesianGame manually =====
    TwoPlayersBayesianGame two_players_bg;

    std::vector<std::vector<int>> types = {{0, 1, 2, 3}, {0, 1, 2, 3}};
    
    TwoPlayersBayesianGame subgame;

    int n1 = types[0].size();
    int n2 = types[1].size();
    int A1 = 2;
    int A2 = 2;

    two_players_bg.setTypeNumbers({std::to_string(n1), std::to_string(n2)});

    // bg.addJointTypeProbabilities({"0.1", "0.2", "0.3", "0.4"}); => it creates a vector 1x4, but we want 2x2 : bg.jointTypeProbabilities = {{0.1, 0.2}, {0.3, 0.4}};
    two_players_bg.addJointTypeProbabilities({"0.16", "0.05", "0.005", "0.005"}); //type 0 of P1 with type 0 -> 3 of P2
    two_players_bg.addJointTypeProbabilities({"0.1", "0.2", "0.01", "0.01"}); //type 1 of P1 with type 0 -> 3 of P2
    two_players_bg.addJointTypeProbabilities({"0.01", "0.02", "0.09", "0.1"}); 
    two_players_bg.addJointTypeProbabilities({"0.02", "0.01", "0.11", "0.1"}); 

    two_players_bg.setGameDimensions({std::to_string(A1), std::to_string(A2)}); //actions

    for (int i =0; i<pow(A1,n1)*pow(A2,n2); i++){
        std::vector<std::string> line;
        int num = rand() % 10 + 1;
        line.push_back(std::to_string(num));
        two_players_bg.addPayoffLine(line);
    }
    
    std::cout<< "Original game joint type probabilities: " << std::endl;
    for (int i=0; i<two_players_bg.jointTypeProbabilities.size(); i++){
        for (int j=0; j<two_players_bg.jointTypeProbabilities[i].size(); j++){
            std::cout << two_players_bg.jointTypeProbabilities[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout<< "\n";

    std::cout << "Subgame joint type probabilities: " << std::endl;
    for (int i=0; i<subgame.jointTypeProbabilities.size(); i++){
        for (int j=0; j<subgame.jointTypeProbabilities[i].size(); j++){
            std::cout << subgame.jointTypeProbabilities[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout<< "\n";
    

    std::vector<std::vector<int>> CK_blocks = build_CK_blocks(&two_players_bg, types, 1, 0.09);

    // visualize CK blocks
    std::cout << "CK blocks: " << std::endl;
    for (int i=0; i<CK_blocks.size(); i++){
        std::cout << "Block " << i << ": "; 
        for (int j=0; j<CK_blocks[i].size(); j++){
            std::cout << CK_blocks[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout<< "\n";

    // decompose game into subgame-mapping pairs
    std::vector<std::pair<SubgameMapping, TwoPlayersBayesianGame>> infoSubgames = decompose_game(&two_players_bg, CK_blocks, types);
    
    // visualize subgames and their mapping tables
    std::cout << "Subgames and Mappings:" << std::endl;
    for (size_t i = 0; i < infoSubgames.size(); i++) {
        const SubgameMapping& mapping = infoSubgames[i].first;
        const TwoPlayersBayesianGame& subgame = infoSubgames[i].second;

        std::cout << "--- Subgame " << i << " ---" << std::endl;
        
        std::cout << "P1 Local-to-Global Map: ";
        for (int global_t1 : mapping.local_to_global_P1) {
            std::cout << global_t1 << " ";
        }
        std::cout << "\nP2 Local-to-Global Map: ";
        for (int global_t2 : mapping.local_to_global_P2) {
            std::cout << global_t2 << " ";
        }
        std::cout << "\nJoint Probabilities:\n";

        for (size_t j = 0; j < subgame.jointTypeProbabilities.size(); j++) {
            for (size_t k = 0; k < subgame.jointTypeProbabilities[j].size(); k++) {
                std::cout << subgame.jointTypeProbabilities[j][k] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "\n";
    }



    // for (int i=0; i<subgames.size(); i++){
    //     std::cout << "Subgame " << i << ": " << std::endl;
    //     for (int j=0; j<subgames[i].jointTypeProbabilities.size(); j++){
    //         for (int k=0; k<subgames[i].jointTypeProbabilities[j].size(); k++){
    //             std::cout << subgames[i].jointTypeProbabilities[j][k] << " ";
    //         }
    //         std::cout << std::endl;
    //     }
    //     std::cout<< "\n";
    // }

    // ===== generate a random BG with the generateRandomBG function from BayesianGameIdenticalPayoff class =====
    // size_t nrAgents = 2;
    // size_t nrActions = 2;
    // size_t nrTypes = 4;
    // BayesianGameIdenticalPayoff global_bg = BayesianGameIdenticalPayoff::GenerateRandomBG(
    //     nrAgents, std::vector<size_t>(2, nrActions), std::vector<size_t>(2, nrTypes)
    // );
    

    //     // visualize the matrix of joint type probabilities
    // std::vector<std::vector<double>> joint_type_probs(global_bg.GetNrTypes(0), std::vector<double>(global_bg.GetNrTypes(1), 0.0));
    // for (size_t jtype = 0; jtype < global_bg.GetNrJointTypes(); ++jtype) {
    //     std::vector<Index> indTypeIndices = global_bg.JointToIndividualTypeIndices(jtype);
    //     joint_type_probs[indTypeIndices[0]][indTypeIndices[1]] = global_bg.GetProbability(jtype);
    // }

    // std::cout << "Joint type probabilities matrix: " << std::endl;
    // for (size_t i = 0; i < joint_type_probs.size(); ++i) {
    //     for (size_t j = 0; j < joint_type_probs[i].size(); ++j) {
    //         std::cout << joint_type_probs[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    // // tranform the global_bg into a TwoPlayersBayesianGame two_players_bg
    // int n1 = (int)global_bg.GetNrTypes(0);
    // int n2 = (int)global_bg.GetNrTypes(1);

    // TwoPlayersBayesianGame two_players_bg;

    // two_players_bg.typesNumbers = { n1, n2 };
    // two_players_bg.gameDimensions = { (int)global_bg.GetNrActions(0), (int)global_bg.GetNrActions(1) };

    // two_players_bg.jointTypeProbabilities = std::vector<std::vector<float>>(n1, std::vector<float>(n2, 0.0f));

    // for (size_t jtype = 0; jtype < global_bg.GetNrJointTypes(); ++jtype) {
    //     std::vector<Index> indTypeIndices = global_bg.JointToIndividualTypeIndices(jtype);
    //     two_players_bg.jointTypeProbabilities[indTypeIndices[0]][indTypeIndices[1]] = global_bg.GetProbability(jtype);
    // }

    // for (size_t jtype = 0; jtype < global_bg.GetNrJointTypes(); ++jtype) {
    //     for (size_t ja = 0; ja < global_bg.GetNrJointActions(); ++ja) {
    //         std::vector<Index> indTypeIndices = global_bg.JointToIndividualTypeIndices(jtype);
    //         std::vector<Index> indActionIndices = global_bg.JointToIndividualActionIndices(ja);
    //         two_players_bg.addPayoffLine({std::to_string(global_bg.GetUtility(indTypeIndices, indActionIndices))});
    //     }
    // }

    // std::cout << "TwoPlayersBayesianGame created from global_bg:" << std::endl;
    // for (size_t i = 0; i < two_players_bg.jointTypeProbabilities.size(); ++i) {
    //     for (size_t j = 0; j < two_players_bg.jointTypeProbabilities[i].size(); ++j) {
    //         std::cout << two_players_bg.jointTypeProbabilities[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    // // decomposition of the BG into subgames
    // std::cout << "Decomposing the BG into subgames..." << std::endl;
    // std::vector<std::vector<int>> types = {{0, 1, 2, 3}, {0, 1, 2, 3}};

    //     // generate the CK blocks using the build_CK_blocks function
    // std::vector<std::vector<int>> CK_blocks = build_CK_blocks(&two_players_bg, types, 1, 0.0872f);
        
    //     // visualize CK blocks
    // std::cout << "CK blocks: " << std::endl;
    // for (int i=0; i<CK_blocks.size(); i++){
    //     std::cout << "CK Block " << i << ": "; 
    //     for (int j=0; j<CK_blocks[i].size(); j++){
    //         std::cout << CK_blocks[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::vector<TwoPlayersBayesianGame> subgames = decompose_game(&two_players_bg, types);
    //     // visualize subgames
    // std::cout << "Subgames: " << std::endl;
    // for (int i=0; i<subgames.size(); i++){
    //     std::cout << "Subgame " << i << ": " << std::endl;
    //     for (int j=0; j<subgames[i].jointTypeProbabilities.size(); j++){
    //         for (int k=0; k<subgames[i].jointTypeProbabilities[j].size(); k++){
    //             std::cout << subgames[i].jointTypeProbabilities[j][k] << " ";
    //         }
    //         std::cout << std::endl;
    //     }
    //     std::cout<< "\n";
    // }  

    // solve each subgame and combine results into global policies
    std::vector<int> global_policy_P1(n1, -1);
    std::vector<int> global_policy_P2(n2, -1);

    std::cout << "Solving subgames with BFS..." << std::endl;
    for (size_t i = 0; i < infoSubgames.size(); i++) {
        const SubgameMapping& mapping = infoSubgames[i].first;
        const TwoPlayersBayesianGame& subgame = infoSubgames[i].second;

        size_t sub_n1 = subgame.jointTypeProbabilities.size();
        size_t sub_n2 = (sub_n1 > 0) ? subgame.jointTypeProbabilities[0].size() : 0;
        
        if (sub_n1 == 0 || sub_n2 == 0) continue; // skip empty subgames

        size_t nrAgents = 2;
        std::vector<size_t> nrActions = {static_cast<size_t>(A1), static_cast<size_t>(A2)};
        std::vector<size_t> nrTypes = {sub_n1, sub_n2};

        // construct MADP BGIP
        BGIP_sharedPtr subgame_ptr(new BayesianGameIdenticalPayoff(nrAgents, nrActions, nrTypes));

        // copy joint probabilities into MADP subgame
        for (size_t t1 = 0; t1 < sub_n1; ++t1) {
            for (size_t t2 = 0; t2 < sub_n2; ++t2) {
                double prob = subgame.jointTypeProbabilities[t1][t2];
                std::vector<Index> type_vec = {(Index)t1, (Index)t2};
                Index jtype = subgame_ptr->IndividualToJointTypeIndices(type_vec);
                subgame_ptr->SetProbability(jtype, prob);
            }
        }

        // copy payoff utilities into MADP subgame
        for (Index jt = 0; jt < subgame_ptr->GetNrJointTypes(); ++jt) {
            for (Index ja = 0; ja < subgame_ptr->GetNrJointActions(); ++ja) {
                subgame_ptr->SetUtility(jt, ja, 1.0);
            }
        }

        // initialize BFS Solver
        BGIP_SolverBruteForceSearch<JointPolicyPureVector> bfs_solver(subgame_ptr, 0, 1);

        // solve subgame
        double subgame_value = bfs_solver.Solve();
        std::cout << "Subgame " << i << " optimal BFS value: " << subgame_value << std::endl;

        const JointPolicyPureVector& sub_policy = bfs_solver.GetJointPolicyPureVector();

        // build global policies
        for (size_t local_t1 = 0; local_t1 < sub_n1; local_t1++) {
            Index best_a1 = sub_policy.GetActionIndex(0, local_t1);
            int global_t1 = mapping.local_to_global_P1[local_t1];
            global_policy_P1[global_t1] = best_a1;
        }

        for (size_t local_t2 = 0; local_t2 < sub_n2; local_t2++) {
            Index best_a2 = sub_policy.GetActionIndex(1, local_t2);
            int global_t2 = mapping.local_to_global_P2[local_t2];
            global_policy_P2[global_t2] = best_a2;
        }
    }

    // visualize combined global policies
    std::cout << "\nCombined global policy from subgames:" << std::endl;
    for (size_t t1 = 0; t1 < n1; ++t1) {
        std::cout << "Player 1 (Type " << t1 << ") => Optimal action: " << global_policy_P1[t1] << std::endl;
    }
    for (size_t t2 = 0; t2 < n2; ++t2) {
        std::cout << "Player 2 (Type " << t2 << ") => Optimal action: " << global_policy_P2[t2] << std::endl;
    }

    // solve original game with BFS to compare
    size_t nrAgents = 2;
    BGIP_sharedPtr original_game_ptr(new BayesianGameIdenticalPayoff(nrAgents, {static_cast<size_t>(A1), static_cast<size_t>(A2)}, {static_cast<size_t>(n1), static_cast<size_t>(n2)}));

    for (size_t t1 = 0; t1 < n1; ++t1) {
        for (size_t t2 = 0; t2 < n2; ++t2) {
            double prob = two_players_bg.jointTypeProbabilities[t1][t2];
            std::vector<Index> type_vec = {(Index)t1, (Index)t2};
            Index jtype = original_game_ptr->IndividualToJointTypeIndices(type_vec);
            original_game_ptr->SetProbability(jtype, prob);
        }
    }

    for (Index jt = 0; jt < original_game_ptr->GetNrJointTypes(); ++jt) {
        for (Index ja = 0; ja < original_game_ptr->GetNrJointActions(); ++ja) {
            original_game_ptr->SetUtility(jt, ja, 1.0);
        }
    }

    BGIP_SolverBruteForceSearch<JointPolicyPureVector> original_bfs_solver(original_game_ptr, 0, 1);

    double original_game_value = original_bfs_solver.Solve();

    std::cout << "\nOriginal game optimal BFS value: " << original_game_value << std::endl;
    for (size_t t1 = 0; t1 < n1; ++t1) {
        Index best_a1 = original_bfs_solver.GetJointPolicyPureVector().GetActionIndex(0, t1);
        std::cout << "Player 1 (Type " << t1 << ") => Optimal action: " << best_a1 << std::endl;
    }
    for (size_t t2 = 0; t2 < n2; ++t2) {
        Index best_a2 = original_bfs_solver.GetJointPolicyPureVector().GetActionIndex(1, t2);
        std::cout << "Player 2 (Type " << t2 << ") => Optimal action: " << best_a2 << std::endl;
    }
    
    // // evaluate the combined strategy on the original BG

    return 0;
}
