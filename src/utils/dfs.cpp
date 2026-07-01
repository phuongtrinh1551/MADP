#include "dfs.hpp"

#define verbose_dfs 0



std::vector<std::vector<support>> getListBlocks(block occ){
    std::vector<std::vector<support>> subgraphs;
    std::unordered_set<support> visited;

    int n=occ.size();
    int p=occ[0].size();

    for (int supportP1=0; supportP1 < occ.size();supportP1++){
	if (!visited.count(supportP1)){
	    std::vector<support> subgraph;
	    dfs(supportP1,1,visited,occ,subgraph);
	    subgraphs.push_back(subgraph);
	}
	//print_blocks(subgraphs);
    }

    for (int supportP2=n; supportP2 < n+occ[0].size();supportP2++){
	if (!visited.count(supportP2)){
	    std::vector<support> subgraph;
	    dfs(supportP2,2,visited,occ,subgraph);
	    subgraphs.push_back(subgraph);
	}
	//print_blocks(subgraphs);
    }
    if (verbose_dfs)
	print_blocks(subgraphs);
    return subgraphs;
}



void dfs(int supportPI, int player, std::unordered_set<support> & visited,block occ, std::vector<support> & current_subgraph){

    //std::cout << "\n player: " << player << "marking PI:" << supportPI;
    visited.insert(supportPI);
    current_subgraph.push_back(supportPI);

    int n=occ.size();
    int p=occ[0].size();

    if (player==1){
	for (int supportP2=n; supportP2<n+occ[0].size(); supportP2++){
	    if (!visited.count(supportP2) && occ[supportPI][supportP2-n]>0.0){
		dfs(supportP2,2,visited,occ,current_subgraph);
	    }
	}

    }

    if (player==2){

	for (int supportP1=0; supportP1<occ.size(); supportP1++){
	    if (!visited.count(supportP1) && occ[supportP1][supportPI-n]>0.0){
		dfs(supportP1,1,visited,occ,current_subgraph);
	    }
	}
    }
}


void print_blocks(std::vector<std::vector<support>> subgraphs){
    std::cout << "\n-------- Printing blocks -----------";
    for (int i=0;i<subgraphs.size();i++){
	std::cout << "\n block:" <<i<< ": {";
	for (int j=0;j<subgraphs[i].size();j++){
	    std::cout << subgraphs[i][j];
	    if (j<subgraphs[i].size()-1)
		std::cout<< ",\t";
	}
	std::cout << "}";
    }
    std::cout << "\n-------------------\n";

}
