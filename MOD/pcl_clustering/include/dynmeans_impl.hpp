/*
Adapted from Trevor Campbell's dynamic-means (https://github.com/trevorcampbell/dynamic-means) by Shih-Yuan Liu
*/

#ifndef __DYNMEANS_IMPL_HPP
template<class Vec>
DynMeans<Vec>::DynMeans(double lambda, double Q, double tau, bool verbose){
  this->verbose = verbose;
  this->lambda = lambda;
  this->Q = Q;
  this->tau = tau;
  this->nextLbl = 0;
  this->observations.clear();
  this->cluster_vec_.clear();
  this->use_alpha_beta_ = false;
}

template<class Vec>
DynMeans<Vec>::~DynMeans(){}

template<class Vec>
void DynMeans<Vec>::reset(){
  this->observations.clear();
  this->nextLbl = 0;
  this->cluster_vec_.clear();  
}

template<class Vec>
void DynMeans<Vec>::setAlphaBeta(double alpha, double beta)
{
  alpha_ = alpha;
  beta_ = beta;
  use_alpha_beta_ = true;
}

//This function is used when sampling parameters - it returns a vector of the observations in the next cluster,
//along with the index of that cluster.
//If this is the last parameter to be sampled, the function returns true; otherwise, false.
template<class Vec>
std::vector<Vec> DynMeans<Vec>::getObsInCluster(int idx, std::vector<int> lbls){
  //std::cout << "Getting obs set in next cluster" << std::endl;
  std::vector<Vec> obsInCluster;
  obsInCluster.reserve(lbls.size());
  //std::cout << "Getting obs for param with idx " << idx << std::endl;
  for (int i = 0; i < lbls.size(); i++){
    if (lbls[i] == idx){
      obsInCluster.push_back(this->observations[i]);
    }
  }
  return obsInCluster;
}

template<class Vec>
std::vector<int> DynMeans<Vec>::updateState(std::vector<int> lbls, std::vector<int> cnts, std::vector<Vec> prms, double current_time){
  /* Update the cluster_map_ using lbls, cnts, and prms */
  /* lbls.size() is the number of observations */
  /* prms.size() and cnt.size() are the number of clusters*/
  // std::cout << "[DynMeans::updateState] Called" << std::endl;

  for (int i = 0; i < prms.size(); ++i){
    if (i < cluster_vec_.size()){
      // Cluster already exist. Update it.
      cluster_vec_[i].update(cnts[i],prms[i],this->tau, current_time);
    }
    else{
      /*New cluster. Push it back, initialize it, and increment nextLbl*/
      cluster_vec_.push_back(Cluster<Vec>());
      cluster_vec_[i].set(cnts[i],prms[i],this->nextLbl++, alpha_, beta_, current_time);
      // this->oldprmlbls.push_back(this->nextLbl);
      // this->nextLbl++;
    }
  }

  std::vector<int> outLbls; //stores the label output using oldprmlbls
  for (int i = 0; i < lbls.size(); i++){
    outLbls.push_back(cluster_vec_[lbls[i]].id_);
  }

  //now that all is updated, check to see which clusters are permanently dead
  for (int i = 0; i < cluster_vec_.size(); i++){
    if (cluster_vec_[i].age_*this->Q > this->lambda){
      // The cluster is permanetly dead
      cluster_vec_.erase(this->cluster_vec_.begin()+i);
      // this->oldprmlbls.erase(this->oldprmlbls.begin()+i);
      i--;
    }
  }

  // std::cout << "[DynMeans::updateState] Finished." << std::endl;
  return outLbls;
}

template<class Vec>
void DynMeans<Vec>::cluster(std::vector<Vec>& newobservations, int nRestarts, 
    std::vector<int>& finalLabels, std::vector<Vec>& finalParams, double& finalObj, double& tTaken, double current_time){
  timeval tStart;
  gettimeofday(&tStart, NULL);

  //set the new obs
  this->observations = newobservations;

  if (newobservations.size() == 0){
    std::cout << "libdynmeans: ERROR: newobservations is empty" << std::endl;
    return;
  }
  if (nRestarts <= 0){
    std::cout << "libdynmeans: ERROR: Cannot have nRestarts <= 0" << std::endl;
    return;
  }

  //generate the orderings
  std::vector< std::vector<int> > randOrderings;
  std::vector<int> tmpindices;
  for (int i = 0; i < newobservations.size(); i++){
    tmpindices.push_back(i);
  }
  std::srand( unsigned( std::time(0) ) );
  for (int i = 0; i < nRestarts; i++){
    //cout << "Generating random order " << i+1 << "/" << nOrderings << "               \r" << flush;
    random_shuffle(tmpindices.begin(), tmpindices.end());
    randOrderings.push_back(tmpindices);
  }

  //stuff for storing best clustering (all other countries make inferior potassium) 
  finalObj =  std::numeric_limits<double>::max();
  std::vector<int> finalCnts;

  if (verbose){
    std::cout << "libdynmeans: Clustering " << newobservations.size() << " datapoints with " << nRestarts << " restarts." << std::endl;
  }

  /* Construct vectors of working variables */
  std::vector<double> obj_vec(nRestarts);
  std::vector< std::vector<Vec> > prms_vec(nRestarts);
  std::vector< std::vector<int> > cnts_vec(nRestarts);
  std::vector< std::vector<int> > lbls_vec(nRestarts);

  /* Parallelize restarts*/
  #pragma omp parallel for
  for (int i = 0; i < nRestarts; i++){
    //create working variables
    std::vector<Vec>& prms = prms_vec[i];
    std::vector<int>& cnts = cnts_vec[i];
    std::vector<int>& lbls = lbls_vec[i];

    for (int j = 0; j < cluster_vec_.size(); j++){
      prms.push_back(cluster_vec_[j].center_); //just placeholders for updated parameters if the old ones get instantiated
      cnts.push_back(0); //start with count 0
    }

    //Initialization: no label on anything
    for (int j = 0; j < this->observations.size(); j++){
      lbls.push_back(-1); // start with no labels on anything
    }

    double obj, prevobj;
    obj = prevobj = std::numeric_limits<double>::max();

    do {
      //do the kmeans iteration
      prevobj = obj;
      this->assignObservations(randOrderings[i], lbls, cnts, prms);
      obj = this->setParameters(lbls, cnts, prms);
      if (obj > prevobj){
        std::cout << "Error: obj > prevobj - monotonicity violated! Check your distance/set parameter functions..." << std::endl;
        std::cout << "obj: " << obj << " prevobj: " << prevobj << std::endl;
      }
      //std::cout << "KMeans -- Restart: " << i+1 << "/" << nRestarts << " Iteration: " << iter << " Objective: " << obj << std::endl;//"\r" << std::flush;
      if (verbose){
        int numinst = 0;
        for (int ii = 0; ii < cnts.size(); ii++){
          if (cnts[ii] > 0){
            numinst++;
          }
        }
        int numnew = prms.size() - cluster_vec_.size();
        int numoldinst = numinst - numnew;
        int numolduninst = cnts.size() - numinst;
      std::cout << "libdynmeans: Trial: " << i+1 << "/" << nRestarts << " Objective: " << obj << " Old Uninst: " << numolduninst  << " Old Inst: " << numoldinst  << " New: " << numnew <<  "                   \r" << std::flush; 
      }
    } while(prevobj > obj);
    /* Save the objective value */
    obj_vec[i] = obj;
  }
  timeval tMid; gettimeofday(&tMid, NULL);
  // std::cout << "[cluster] mid took " << getDuration(tStart,tMid) << std::endl;

  /* Find the index with the minimum objective value */
  std::vector<double>::iterator it_obj = std::min_element(obj_vec.begin(),obj_vec.end());
  int min_obj_index = it_obj - obj_vec.begin();
  /* Populate output */
  finalObj = obj_vec[min_obj_index];
  finalParams = prms_vec[min_obj_index];
  finalLabels = lbls_vec[min_obj_index];
  finalCnts   = cnts_vec[min_obj_index];

  if (verbose){
    int numinst = 0;
    for (int ii = 0; ii < finalCnts.size(); ii++){
      if (finalCnts[ii] > 0){
        numinst++;
      }
    }
    int numnew = finalParams.size() - cluster_vec_.size();
    int numoldinst = numinst - numnew;
    int numolduninst = finalCnts.size() - numinst;
    std::cout << "libdynmeans: Done clustering. Min Objective: " << finalObj << " Old Uninst: " << numolduninst  << " Old Inst: " << numoldinst  << " New: " << numnew <<  std::endl;
  }



  //update the stored results to the one with minimum cost
  if (use_alpha_beta_){
    finalLabels = this->updateState(finalLabels, finalCnts, finalParams, current_time);
  }
  else{
    finalLabels = this->updateState(finalLabels, finalCnts, finalParams, 0.0);
  }


  timeval tEnd; gettimeofday(&tEnd, NULL);
  tTaken = getDuration(tStart,tEnd);
  // std::cout << "[DynMeans::cluster] Finished." << std::endl;
  // std::cout << "[cluster] end took " << tTaken << std::endl;
  return;
}

template<class Vec>
double DynMeans<Vec>::getDuration(const timeval& tStart, const timeval& tCur)
{
  return (double)(tCur.tv_sec - tStart.tv_sec) + (double)(tCur.tv_usec - tStart.tv_usec)/1.0e6;
}



template<class Vec>
void DynMeans<Vec>::assignObservations(std::vector<int> assgnOrdering, std::vector<int>& lbls, std::vector<int>& cnts, std::vector<Vec>& prms){
  // std::cout << "[DynMeans::assignObservations] Started." << std::endl;
  for (int i = 0; i < assgnOrdering.size(); i++){
    //get the observation idx from the random ordering
    int idx = assgnOrdering[i];

    //store the old lbl for possibly deleting clusters later
    int oldlbl = lbls[idx];
    
    std::vector<double> distsq_vec(prms.size());   
    // std::cout << "prms size: " << prms.size() << std::endl;
    #pragma omp parallel for
    for (int j = 0; j < prms.size(); j++){
      // double &tmpdistsq = distsq_vec[j];
      double tmpdistsq = (prms[j] - this->observations[idx]).squaredNorm();
      if (cnts[j] == 0){//the only way cnts can get to 0 is if it's an old parameter
        double gamma = 1.0/(1.0/cluster_vec_[j].weight_ + cluster_vec_[j].age_*this->tau);
        tmpdistsq = gamma/(1.0+gamma)*tmpdistsq + cluster_vec_[j].age_*this->Q;
      }
      distsq_vec[j] = tmpdistsq;  
    }

    /* Find minind */
    // std::cout << "==== HERE ==== " << std::endl;
    int minind = 0;
    double mindistsq = std::numeric_limits<double>::max();
    std::vector<double>::iterator distsq_it = std::min_element(distsq_vec.begin(),distsq_vec.end());
    if (distsq_it != distsq_vec.end()){
      minind = distsq_it - distsq_vec.begin();
      mindistsq = *distsq_it;
    }

    //if the minimum distance is stil greater than lambda + startup cost, start a new cluster
    if (mindistsq > this->lambda){
      prms.push_back(this->observations[idx]);
      lbls[idx] = prms.size()-1;
      cnts.push_back(1);
    }
    else{
      if (cnts[minind] == 0){
      /*if we just instantiated an old cluster, update its parameter to the current timestep so that upcoming assignments are valid*/
        double gamma = 1.0/(1.0/cluster_vec_[minind].weight_ + cluster_vec_[minind].age_*this->tau);
        prms[minind] = (cluster_vec_[minind].center_*gamma + this->observations[idx])/(gamma + 1);
      }
      lbls[idx] = minind;
      cnts[minind]++;
    }

    //if obs was previously assigned to something, decrease the count the clus it was assigned to
    //we do cluster deletion *after* assignment to prevent corner cases with monotonicity
    if (oldlbl != -1){
      cnts[oldlbl]--;
      //if this cluster now has no observations, but was a new one (no age recording for it yet)
      //remove it and shift labels downwards
      if (cnts[oldlbl] == 0 && oldlbl >= cluster_vec_.size()){
        prms.erase(prms.begin() + oldlbl);
        cnts.erase(cnts.begin() + oldlbl);
        for (int j = 0; j < lbls.size(); j++){
          if (lbls[j] > oldlbl){
            lbls[j]--;
          }
        }
      } else if (cnts[oldlbl] == 0){//it was an old parameter, reset it to the oldprm
        prms[oldlbl] = cluster_vec_[oldlbl].center_;
      }
    }
  }
  // std::cout << "[DynMeans::assignObservations] Ended." << std::endl;
  return; 
}

template<class Vec>
double DynMeans<Vec>::setParameters(std::vector<int>& lbls, std::vector<int>& cnts, std::vector<Vec>& prms){
  // double objective = 0;
  std::vector<double> obj_vec(prms.size(),0.0);
  #pragma omp parallel for
  for (int i = 0; i < prms.size(); i++){
    double& objective = obj_vec[i];
    if (cnts[i] > 0){
      //add cost for new clusters - lambda
      // or add cost for old clusters - Q
      if (i < cluster_vec_.size()){
        // objective += this->Q*this->ages[i];
        objective += this->Q*cluster_vec_[i].age_;
      } else {
        objective += this->lambda;
      }
      std::vector<Vec> obsInClus = this->getObsInCluster(i, lbls);
      Vec tmpvec = obsInClus[0];
      for (int j = 1; j < obsInClus.size(); j++){
        tmpvec = tmpvec + obsInClus[j];
      }
      tmpvec = tmpvec / obsInClus.size();
      if (i < cluster_vec_.size()){ //updating an old param
        // double gamma = 1.0/(1.0/this->weights[i] + this->ages[i]*this->tau);
        double gamma = 1.0/(1.0/cluster_vec_[i].weight_ + cluster_vec_[i].age_*this->tau);
        prms[i] = (cluster_vec_[i].center_*gamma + tmpvec*cnts[i])/(gamma + cnts[i]);
        //add parameter lag cost
        double tmpsqdist = (prms[i] - cluster_vec_[i].center_).squaredNorm();
        objective += gamma*tmpsqdist;
      } else { //just setting a new param
        prms[i] = tmpvec;
        //no lag cost for new params
      }
      //get cost for prms[i]
      for (int j = 0; j < obsInClus.size(); j++){
        objective += (prms[i] - obsInClus[j]).squaredNorm();
      }
    }
  }
  if (obj_vec.size() > 0){
    return std::accumulate(obj_vec.begin(), obj_vec.end(), 0.0);
  }
  else{
    return 0;
  }
}


#define __DYNMEANS_IMPL_HPP
#endif /* __DYNMEANS_IMPL_HPP */
