#include<string>
#include <vector>
#include <map>
using namespace std;

// to use sleep (different import on windows or ubuntu)
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rclcpp/rclcpp.hpp"

#include "delta_interfaces/msg/level_objects.hpp"

#include "delta_interfaces/msg/job_status.hpp"
#include "delta_interfaces/msg/greeter_job.hpp"
#include "delta_interfaces/msg/explorer_job.hpp"


enum MissionStatus { ready, continueExploring, stopExploring, greetingPerson };

class MissionController : public rclcpp::Node {
  public:
    MissionController() : Node("mission_controller") {
       // getting level objects
       _levelObjectsSuscription = this->create_subscription<delta_interfaces::msg::LevelObjects>(
         "level_objects", 1, std::bind(&MissionController::receiveLevelObjectsUpdate, this, std::placeholders::_1));
        
      // receiving job status
      _jobStatusSuscription = this->create_subscription<delta_interfaces::msg::JobStatus>(
        "job_status", 1, std::bind(&MissionController::receiveJobStatusUpdate, this, std::placeholders::_1));
        
      // sending greeter jobs
      _greeterJobPublisher = this->create_publisher<delta_interfaces::msg::GreeterJob>("greeter_job", 1);
      
      // sending explorer jobs
      _explorerJobPublisher = this->create_publisher<delta_interfaces::msg::ExplorerJob>("explorer_job", 1);
      
      _decisionTimer = this->create_wall_timer(500ms, std::bind(&MissionController::makeDecision, this));
    }
  private:
    rclcpp::TimerBase::SharedPtr _decisionTimer;
    mutable MissionStatus _myStatus = ready;
    
    // level objects
    rclcpp::Subscription<delta_interfaces::msg::LevelObjects>::SharedPtr _levelObjectsSuscription;
    mutable struct {
      vector<double> x;
      vector<double> y;
      vector<double> z;
      vector<double> rot;
      vector<string> id;
      int numberOfObjects = 0;
    } _levelObjects;
    mutable map<string, bool> _knownLevelObjectIds = {};
    mutable vector<string> _peopleStillToGreetIds;
    
    // sending jobs to servants stuff
    mutable bool _servantReceivedJob = false;
    mutable bool _servantHasFinishedJob = false;
    mutable string _sentJobId = ""; // the id of the job that was last sent
    mutable int _jobCounter = 0; // makes sure each job has an individual id
    
    // getting job status updates
    rclcpp::Subscription<delta_interfaces::msg::JobStatus>::SharedPtr _jobStatusSuscription;
    
    // explorer stuff
    rclcpp::Publisher<delta_interfaces::msg::ExplorerJob>::SharedPtr _explorerJobPublisher;
    
    // greeting stuff
    const int _peopleToGreetForVictory = 3;
    mutable int _greetedPeople = 0;
    mutable string _nextPersonToGreetId = "";
    rclcpp::Publisher<delta_interfaces::msg::GreeterJob>::SharedPtr _greeterJobPublisher;
    
    
    //i need to remember in a bool if the sent job has ever arrived -> even if it changes then i know the job is completed.
    //this bool has to be reseted to false every time a new job has to be sent!!
    //i dont need to store the _greeterCurrentJobId, but i can just set the mentioned bool whenever a message arrives!
    //actually _sentJobId can be the same for greeting and start and stop exploring
    
    void initNewJobId() {
        _sentJobId = "mission_control_job"+ to_string(_jobCounter);
        _jobCounter++;
        _servantReceivedJob = false;
        _servantHasFinishedJob = false;
    }
    
    
    void makeDecision() {
    
        if(missionComplete()) {
            displayVictoryText();
            _decisionTimer->cancel();
            return;
        }
        
        //RCLCPP_INFO(this->get_logger(), "servant - received job: '%d'", _servantReceivedJob);
        //RCLCPP_INFO(this->get_logger(), "servant - has finished job: '%d'", _servantHasFinishedJob);
        
        // the robot is ready to perform the next best action
        if (_myStatus == ready) {

            if (needToGreet()) {
                // need to greet someone!
                switchStatus(greetingPerson);
                
            } else {
                // i dont need to greet anyone so i go exlore =)
                switchStatus(continueExploring);
            }
        }
        
        // the robot has to explore right now
        else if (_myStatus == continueExploring) {
        
            // the exploration job has not arrived at the robot yet
            if (!_servantReceivedJob) {
                // send exploring job
                RCLCPP_INFO(this->get_logger(), "continueExploring : sending job");
                sendContinueExplorationJob();
            
            // the exploration job has arrived but was not executed yet
            } else if (!_servantHasFinishedJob) {
                // wait for exploration job to be executed
                RCLCPP_INFO(this->get_logger(), "continueExploring : waiting for explorer to start exploring");
            
            // the robot is exploring right now
            } else {
                if (needToGreet()) {
                    // robot has to greet -> switch to stop exploration status
                    RCLCPP_INFO(this->get_logger(), "continueExploring : need to greet someone, stopping exploration");
                    switchStatus(stopExploring);
                
                // the robot is exploring as it should. nothing to do here
                } else {
                    RCLCPP_INFO(this->get_logger(), "continueExploring : I am exploring right now!");
                }
            }
        }
        
        // the robot has to stop the exploration
        else if (_myStatus == stopExploring) {
        
            if (!_servantReceivedJob) {
                // send stop exploring command again
                RCLCPP_INFO(this->get_logger(), "stopExploring : sending job");
                sendStopExplorationJob();
            
            } else if (!_servantHasFinishedJob) {
                // stop exploring command has arrived, but not executed yet -> nothing to do
                RCLCPP_INFO(this->get_logger(), "stopExploring : waiting for explorer to stop exploring");
            
            } else { // stop command arrived and was executed
                // not exploring anymore -> get into ready status
                RCLCPP_INFO(this->get_logger(), "stopExploring : I stopped exploring");
                switchStatus(ready);
            }
        
        }
        
        
        // the robot is currently greeting a person
        else if (_myStatus == greetingPerson) {
        
            // greeter has not received job yet -> sending job
            if (!_servantReceivedJob) {
                RCLCPP_INFO(this->get_logger(), "greetingPerson : sending job");
                sendGreeterJob();
            }
            
            // greeter has already received job but not finished yet
            else if (!_servantHasFinishedJob) {
                // give greeter time to to greeting
                RCLCPP_INFO(this->get_logger(), "greetingPerson : waiting for greeter to finish greeting");
                
            } else {
                // greeter is done. ready for new task
                RCLCPP_INFO(this->get_logger(), "greetingPerson : greeting finished. getting ready for new task");
                _greetedPeople++;
                switchStatus(ready);
            }
        }
    }
    
    bool missionComplete() {
        return _greetedPeople >= _peopleToGreetForVictory;
    }
    
    void displayVictoryText() {
        RCLCPP_INFO(this->get_logger(), "I greeted all %s people. My mission is complete!!!", to_string(_peopleToGreetForVictory).c_str());
        sleep(5);
        RCLCPP_INFO(this->get_logger(), "I am so proud :,)");
        sleep(3);
        RCLCPP_INFO(this->get_logger(), "Wish my robo mum could see this");
        sleep(4);
        RCLCPP_INFO(this->get_logger(), "But she...");
        sleep(1);
        RCLCPP_INFO(this->get_logger(), "was...");
        sleep(2);
        RCLCPP_INFO(this->get_logger(), "...recycled into a toaster");
        sleep(4);
        RCLCPP_INFO(this->get_logger(), " ");
        sleep(2);
        RCLCPP_INFO(this->get_logger(), "Life is cruel.");
        sleep(5);
        RCLCPP_INFO(this->get_logger(), "Leave me alone now please");
    }
    
    void switchStatus(MissionStatus newStatus) {
        _myStatus = newStatus;
        
        if (newStatus == ready) {
        
        } else if (newStatus == continueExploring) {
            initNewJobId();
            RCLCPP_INFO(this->get_logger(), "continueExploring : sending job");
            sendContinueExplorationJob();
            
        } else if (newStatus == stopExploring) {
            initNewJobId();
            RCLCPP_INFO(this->get_logger(), "stopExploring : sending job");
            sendStopExplorationJob();
            
        } else if (newStatus == greetingPerson) {
            initNewJobId();
            _nextPersonToGreetId = popPersonToGreetAndReturnId();
            RCLCPP_INFO(this->get_logger(), "greetingPerson : sending job");
            sendGreeterJob();
        }
    }
    
    bool needToGreet() {
        return _peopleStillToGreetIds.size() > 0;
    }
    
    string popPersonToGreetAndReturnId() {
        if (!needToGreet()) RCLCPP_INFO(this->get_logger(), "ERROR: sendGreeterJob() was called but there are no people left in _peopleStillToGreetIds");
        
        //int lastIndex = _peopleStillToGreetIds.size() - 1; // get and remove last element
        string personToGreetId = _peopleStillToGreetIds.back(); //[lastIndex];
        _peopleStillToGreetIds.pop_back();  //.erase(lastIndex);
        
        return personToGreetId;
    }
    
    void sendGreeterJob() {
        for (int i = 0; i < _levelObjects.numberOfObjects; i++) {
            if (_levelObjects.id[i] != _nextPersonToGreetId) continue;
            
            auto message = delta_interfaces::msg::GreeterJob();
            message.position_x = _levelObjects.x[i];
            message.position_y = _levelObjects.y[i];
            message.position_z = _levelObjects.z[i];
            message.rotation = _levelObjects.rot[i];
            message.job_id = _sentJobId;
            _greeterJobPublisher->publish(message);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "ERROR: person that has to be greeted is not in the list of all level object");
    }
    
    void sendContinueExplorationJob() {
        sendExplorerJob(true);
    }
    
    void sendStopExplorationJob() {
        sendExplorerJob(false);
    }
    
    void sendExplorerJob(bool explore) {
        auto message = delta_interfaces::msg::ExplorerJob();
        message.explore = explore;
        message.job_id = _sentJobId;
        _explorerJobPublisher->publish(message);
    }
    
    void receiveJobStatusUpdate(const delta_interfaces::msg::JobStatus & msg) const {
    
        // the job has arrived
        if (msg.job_id == _sentJobId) {
            _servantReceivedJob = true;
        }
        
        // the job has arrived and is eighter not processed anymore -> finished
        if (msg.job_id == _sentJobId && (!msg.acting)) {
            _servantHasFinishedJob = true;
        }
    }

    void receiveLevelObjectsUpdate(const delta_interfaces::msg::LevelObjects & msg) const {
      //vector<double> x = msg.position_x;
      //vector<double> y = msg.position_y;
      //vector<double> z = msg.position_z;
      //vector<double> rot = msg.rotation;
      //vector<string> id = msg.id;
      //int numberOfObjects = msg.number_of_objects;
      
      bool nothingNew = true;
      
      for (int i = 0; i < msg.number_of_objects; i++) {
          string id = msg.id[i];
          if (_knownLevelObjectIds.count(id) == 0) { // check if _knownLevelObjectIds is not containing msg.id[i] yet
              _knownLevelObjectIds[id] = true; // add to map to keep track which object ids are known already
              
              // all objects are people for now
              _peopleStillToGreetIds.push_back(id); // add it to list of people that the robot still needs to greet
              
              _levelObjects.x.push_back(msg.position_x[i]); // add unknown level object to all level objects
              _levelObjects.y.push_back(msg.position_y[i]);
              _levelObjects.z.push_back(msg.position_z[i]);
              _levelObjects.rot.push_back(msg.rotation[i]);
              _levelObjects.id.push_back(msg.id[i]);
              _levelObjects.numberOfObjects++;
              
              nothingNew = false;
          }
      }
      
      if (nothingNew) return;
      
      RCLCPP_INFO(this->get_logger(), "_____________________________________________");
      RCLCPP_INFO(this->get_logger(), "I heard a list of %d level objects. My objects are now:", msg.number_of_objects);
      for (int i = 0; i < _levelObjects.numberOfObjects; i++) {
      RCLCPP_INFO(this->get_logger(), "----------------------------");
        RCLCPP_INFO(this->get_logger(), "I heard id: '%s'", (_levelObjects.id[i]).c_str());
        RCLCPP_INFO(this->get_logger(), "I heard x: '%f'", _levelObjects.x[i]);
        RCLCPP_INFO(this->get_logger(), "I heard y: '%f'", _levelObjects.y[i]);
        RCLCPP_INFO(this->get_logger(), "I heard z: '%f'", _levelObjects.z[i]);
        RCLCPP_INFO(this->get_logger(), "I heard r: '%f'", _levelObjects.rot[i]);
      }
      
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionController>());
  rclcpp::shutdown();
  return 0;
}
