#include "graph.h"
void Graph::Establish(cv::Mat& vertices)
{
	// Initialize
	vertices_=vertices;	   	// record raw vertices matrix
	vcols_=vertices.cols; 	// width of raw vertices matrix
	vrows_=vertices.rows;	
	
	number_of_vertices_=vertices.cols*vertices.rows;	// Get the number of vertices
	
	// Init is_visited list
	is_visited_.resize(number_of_vertices_);
	if(is_visited_[0]!=false){
		for(int i=0;i<is_visited_.size();i++)
			is_visited_[i]=false;
	}		
	
	// 1st pixel
	if(vertices.at<uchar>(0,0)==0)
		v_active_.push_back(Element(0,0));
	else
		is_visited_[0]=true;
		
	// 1st row
	for(int j=1;j<vcols_;j++) {
		if(vertices.at<uchar>(0,j)==0){
			// update vertex list
			v_active_.push_back(Element(0,j));
			if(vertices.at<uchar>(0,j-1)==0){
				int idx1=cvt(0,j-1);
				int idx2=cvt(0,j);
				Insert(idx1,idx2);
				Insert(idx2,idx1);
			}
		}
		else
			is_visited_[cvt(0,j)]=true;
	}
	// 1st column
	for(int i=1;i<vrows_;i++){
		if(vertices.at<uchar>(i,0)==0){
			// update vertex list
			v_active_.push_back(Element(i,0));
			if(vertices.at<uchar>(i-1,0)==0){
				int idx1=cvt(i-1,0);
				int idx2=cvt(i,0);
				Insert(idx1,idx2);
				Insert(idx2,idx1);				
			}
		}
		else
			is_visited_[cvt(i,0)]=true;
	}
	
	for(int i=1;i<vertices.rows;i++){
		for(int j=1;j<vertices.cols;j++){
			if(vertices.at<uchar>(i,j)==0){ // if current i,j is a vertice
			
				// update vertex
				v_active_.push_back(Element(i,j));
				
				// define
				int idx_top_left=cvt(i-1,j-1);
				int idx_top=cvt(i-1,j);
				int idx_left=cvt(i,j-1);
				int idx_focus=cvt(i,j);
				
				if(vertices.at<uchar>(i-1,j-1)==0){ // top left pixel
					// record edges					
					Insert(idx_top_left,idx_focus);
					Insert(idx_focus,idx_top_left);
				}
				if(vertices.at<uchar>(i-1,j)==0){
					// record edges
					Insert(idx_top,idx_focus);
					Insert(idx_focus,idx_top);				
				}
				if(vertices.at<uchar>(i,j-1)==0){
					// record edges
					Insert(idx_left,idx_focus);
					Insert(idx_focus,idx_left);
				}
			}
			else
				is_visited_[cvt(i,j)]=true;
		}
	}
}

int Graph::BFS(int start_vertex,Result& rst_tmp)
{
	int count=0;
	queue<int> Q;
	Q.push(start_vertex);
	rst_tmp.Insert(start_vertex,vcols_);
	count++;
	is_visited_[start_vertex]=true; // mark it is visited
	while(!Q.empty()){
		// get the top value
		int i=Q.front();
		Q.pop();
		// visit neighours
		for(int j=0;j<number_of_vertices_;j++){
			if(is_visited_[j]==false && Search(i,j)==true){
				count++;
				is_visited_[j]=true;
				Q.push(j);
				rst_tmp.Insert(j,vcols_);
			}
		}
	}
	return count;
}

void Graph::Run()
{
	/* cout<<endl<<"Running..."<<endl; */
	for(int i=0;i<is_visited_.size();i++)
	{
		if(is_visited_[i]==false)
		{
			Result rst_tmp;
			rst_tmp.number_ = BFS(i,rst_tmp);
			rst_.push_back(rst_tmp);
		}
	}
	sort(rst_.begin(),rst_.end(),[](const Result& e1,const Result& e2){ return e1.number_ > e2.number_;});
	
	/* cv::imshow("2D Viewer",vertices_);
	cv::imwrite("tmp.bmp",vertices_);
	cv::waitKey(0); 	
	cout<<"Done!"<<endl;*/
}

void Graph::Print()
{
	cout<<endl<<"rows:"<<vrows_<<endl;
	cout<<"cols:"<<vcols_<<endl;
	
	for(int i=0;i<v_active_.size();i++)
	{
		printf("(%d,%d)",v_active_[i].i_,v_active_[i].j_);
	}
	cout<<endl;
	
	cout<<"map list"<<endl;
	for(int i=0;i<map_list_.size();i++)
	{
		EdgeNode* p=map_list_[i].next_;
		printf("%d ",map_list_[i].id_);
		while(p!=NULL){
			printf("%d ",p->id_);
			p=p->next_;
		}
		cout<<endl;
	}
}