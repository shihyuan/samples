/*
Adapted from Trevor Campbell's dynamic-means (https://github.com/trevorcampbell/dynamic-means) by Shih-Yuan Liu
*/
#ifndef __DYNMEANS_HPP
#include <vector>
#include <iostream>
#include <algorithm>
#include <boost/static_assert.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <sys/time.h>
#include <ctime>
#include <map>
#include <numeric>

template <class Vec>
class Cluster{
public:
	Cluster(){}
	~Cluster(){}
	int id_;
	Vec center_;
	double weight_;
	int age_;
	Vec velocity_;

	double alpha_;
	double beta_;
	double last_update_time_;

	void update(int count, Vec center, double tau, double current_time)
	{
		if (count > 0){
			weight_ = 1.0/(1.0/weight_ + age_*tau) + count;
			age_ = 0;
			if (current_time > 0.0){
				/* Update velocity and center using the alpha-beta filter*/
				double dt = std::max(current_time - last_update_time_,0.01); //Avoid divid by zero.

				Vec center_est = center_ + dt*velocity_;
				Vec center_err = center - center_est;
				center_ = center_est + alpha_*center_err;
				velocity_ = velocity_ + (beta_/dt)*center_err;
				last_update_time_ = current_time;
			}
			else{
				/* No filtering */
				center_ = center;
			}
		}
		age_++;
	}
	void set(int count, Vec center, int id, double alpha, double beta, double current_time){
		weight_ = count;
		age_ = 0;
		center_ = center;
		id_ = id;
		alpha_ = alpha;
		beta_ = beta;
		last_update_time_ = current_time;
		velocity_ = Vec::Zero(); //Initialize velocity to zero. 
	}
};


template <class Vec>
class DynMeans{
	public:
		DynMeans(double lambda, double Q, double tau, bool verbose = false);
		~DynMeans();
		//initialize a new step and cluster
		void cluster(std::vector<Vec>& newobservations, int nRestarts, std::vector<int>& finalLabels, std::vector<Vec>& finalParams, double& finalObj, double& tTaken, double current_time);
		//reset DDP chain
		void reset();
		// std::vector<Vec> oldprms;
		// std::vector<int> oldprmlbls;
		std::vector<Cluster<Vec> > cluster_vec_;
		void setAlphaBeta(double alpha, double beta);
	private:
		double lambda, Q, tau;
		bool verbose;
		std::vector<Vec> observations;
		//during each step, constants which are information about the past steps
		//once each step is complete, these get updated
		int nextLbl;
		double alpha_, beta_;
		bool use_alpha_beta_;
		// std::vector<double> weights;
		// std::vector<int> ages;

		//tools to help with kmeans
		std::vector<Vec> getObsInCluster(int idx, std::vector<int> lbls); 
		void assignObservations(std::vector<int> assgnOrdering, std::vector<int>& lbls, std::vector<int>& cnts, std::vector<Vec>& prms);
		double setParameters(std::vector<int>& lbls, std::vector<int>& cnts, std::vector<Vec>& prms);
		std::vector<int> updateState(std::vector<int> lbls, std::vector<int> cnts, std::vector<Vec> prms, double current_time = 0.0);
		double getDuration(const timeval& tStart, const timeval& tCur);
};
#include "dynmeans_impl.hpp"
#define __DYNMEANS_HPP
#endif /* __DYNMEANS_HPP */
