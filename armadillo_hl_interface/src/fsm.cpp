#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

#include <armadillo_hl_interface/fsm.h>

// FSM Implementation

FSM::FSM():
    _nodes(),
    _start(2)
{ }

int FSM::execute(){
    int id = _start;
    while(id && ros::ok()){
        if(id == 1)
            return id;
        ROS_INFO("executing node %i", id);
        id = _nodes.at(id)->execute();
    }

    if(!ros::ok())
        return 0;

    return id;
}

bool FSM::run(){
    return execute();
}

void FSM::set_start(int id){
    _start = id;
}

void FSM::add_node(int id, FSMNode *node){
    if(!_nodes.insert(std::pair<int, FSMNode*>(id, node)).second)
        ROS_ERROR("node %i was already defined!", id);
}

FSM::~FSM(){}

// FuncFSMNode Implementation

FuncFSMNode::FuncFSMNode(FuncFSMNode::ExecuteFunc func):
    _func(func)
{}

int FuncFSMNode::execute(){
    return _func();
}

FuncFSMNode::~FuncFSMNode(){}

// DisjFSMNode Implementation

DisjFSMNode::DisjFSMNode(const std::vector<FSMNode*> nodes):
    _nodes(nodes),
    _worker_mutex(),
    _counter(0),
    _next(0),
    _done(true)
{}

void DisjFSMNode::worker(FSMNode *node){
    int next = node->execute();
    _worker_mutex.lock();
    if(!_done){
        _next = next;
        _counter.post();
        _done = true;
    }
    _worker_mutex.unlock();
}

int DisjFSMNode::post_execution(int next){
    return next;
}

int DisjFSMNode::execute(){
    _done = false;
    
    // start threads
    for(std::vector<FSMNode*>::const_iterator it = _nodes.begin(); it != _nodes.end(); ++it){
        boost::thread(&DisjFSMNode::worker, this, *it);
    }

    // wait for first thread to finish
    _counter.wait();

    // execute post-execution method and return
    return post_execution(_next);
}

DisjFSMNode::~DisjFSMNode(){

}

// ConjFSMNode Implementation

ConjFSMNode::ConjFSMNode(std::vector<FSMNode*> nodes):
        _nodes(nodes),
        _next(nodes.size()),
        _counter(0)
    {}

int ConjFSMNode::post_execution(std::vector<int> &next){
    if(next.empty())
        return 0;
    else
        return next[0];
}

void ConjFSMNode::worker(int id){
    // execute node
    _next[id] = _nodes[id]->execute();

    // inc counter
    _counter.post();
}

int ConjFSMNode::execute(){
    // start all threads
    for(int i=0; i<_nodes.size(); i++)
        boost::thread(&ConjFSMNode::worker, this, i);

    // wait for all threads to finish
    for(int i=0; i<_nodes.size(); i++)
        _counter.wait();

    // run post-execution method and return
    return post_execution(_next);
}

ConjFSMNode::~ConjFSMNode(){

}