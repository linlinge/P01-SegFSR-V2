#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include <queue>
#include <stdio.h>
#include "V2I.hpp"
using namespace std;
class Element
{
	public:
		int i_;
		int j_;
		int category_;
		Element(){};
		Element(int i,int j){
			i_=i;
			j_=j;
			category_=INT_MAX;
		}
		void Init(int i,int j){
			i_=i;
			j_=j;
			category_=INT_MAX;
		}
};

class EdgeNode{
	public:
		int id_;
		EdgeNode* next_;		
};

class VertexNode{
	public:
		int id_;
		EdgeNode* next_;
		VertexNode(int id){
			id_=id;
			next_=NULL; 
		}
		// at here
		void Insert(int id){
			EdgeNode* p=new EdgeNode;
			p->id_=id;
			p->next_=next_;
			next_=p;
		}	
};
class Result{
	public:
		int number_;
		vector<V2I> pLoc_;
		void Insert(int val,int& cols){
			number_++;
			V2I tmp;
			tmp.i_=val/cols;
			tmp.j_=val%cols;			
			pLoc_.push_back(tmp);
		}		
};

class Graph
{
	public:
		vector<VertexNode> map_list_;
		cv::Mat vertices_;
		int vcols_; // use to record columns of raw vertex matrix
		int vrows_;
		int number_of_vertices_;
		vector<Element> v_lock_;
		vector<Element> v_active_;
		vector<bool> is_visited_;
		vector<Result> rst_;
		
		
		void Establish(cv::Mat& vertices);
		int BFS(int start_vertex,Result& rst_tmp);
		void Run();
		void Print(); 
		
		int cvt(int i,int j){ return i*vcols_+j;}
		void cvt(int indices,int& i,int& j){ i=floor(indices*1.0/vcols_); j=indices-i*vcols_;}
		
		// map_list_ operation function
		int GetNodeById(int id){
			for(int i=0;i<map_list_.size();i++){
				if(map_list_[i].id_==id)
					return i;				
			}
			return -1;
		}
		void Insert(int id1,int id2){
			int pos=GetNodeById(id1);
			if(pos==-1){// not found
				VertexNode node_tmp(id1);
				node_tmp.Insert(id2);
				map_list_.push_back(node_tmp);
			}
			else{ // found
				map_list_[pos].Insert(id2);
			}
		}
		bool Search(int id1,int id2){
			int pos=GetNodeById(id1);
			if(pos==-1) // id1 do not have neighours
				return false;
			else{
				EdgeNode* p=map_list_[pos].next_;
				while(p!=NULL){
					if(p->id_== id2)
						return true; // contained in id1's neighbour
					p=p->next_;
				}
				return false; // do not have this neighour
			}
		}
};