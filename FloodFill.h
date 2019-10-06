#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <omp.h>
using namespace std;

class Vertex
{
	public:
		int i_,j_;
		Vertex* next;
};

class Vertices
{
	public:
		Vertex* head_;
		int number_;
		
		Vertices(){
			Vertex* p=new Vertex;
			p->i_=INT_MAX;
			p->j_=INT_MAX;
			head_=p;
			head_->next=NULL;
			number_=0;
		}
		void Insert(int i,int j){
			Vertex* p=new Vertex;
			p->i_=i;
			p->j_=j;
			p->next=head_->next;
			head_->next=p;
			number_++;
		}
		void Insert(Vertex* p){
			p->next=head_->next;
			head_->next=p;
			number_++;
		}
		int Delete(int i,int j){
			Vertex* p=head_;
			Vertex* q=NULL;
			
			while(p->next!=NULL){
				if(p->next->i_==i && p->next->j_==j)
					break;
				p=p->next;					
			}
			if(p->next==NULL)
				return -1;
			else{
				q=p->next;
				p->next=q->next;
				delete q;
				number_--;
				return 1;
			}
		}
		Vertex* Remove(int i,int j){
			Vertex* p=head_;
			Vertex* q=NULL;
			while(p->next!=NULL){
				if(p->next->i_==i && p->next->j_==j)
					break;
				p=p->next;					
			}
			if(p->next==NULL)
				return NULL;
			else{
				q=p->next;
				p->next=q->next;
				number_--;
				return q;
			}
		}
};

class FloodFill
{
	public:
		Vertices active_vertices_;	// mark the vertices which are not be used
		vector<Vertices> result_;
		
		// External Function
		FloodFill(cv::Mat& img);
		
		// Internal Function
		void floodFillUtil(cv::Mat& screen, int x, int y, Vertices& ant);
};