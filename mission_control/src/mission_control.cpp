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
#include "delta_interfaces/msg/cylinder_objects.hpp"
#include "delta_interfaces/msg/ring_objects.hpp"
#include "delta_interfaces/msg/say_text.hpp"

#include "delta_interfaces/msg/job_status.hpp"
#include "delta_interfaces/msg/greeter_job.hpp"
#include "delta_interfaces/msg/explorer_job.hpp"
#include "delta_interfaces/msg/parking_job.hpp"
#include "delta_interfaces/msg/monalisa_job.hpp"


enum MissionStatus { ready, continueExploring, stopExploring, greetingPerson, parking, scanningQr, approachingMonalisa, checkMonalisa };

class MissionController : public rclcpp::Node {
  public:
    MissionController() : Node("mission_controller") {
       // getting level objects
       _levelObjectsSuscription = this->create_subscription<delta_interfaces::msg::LevelObjects>(
         "level_objects", 1000, std::bind(&MissionController::receiveLevelObjectsUpdate, this, std::placeholders::_1)); 
       // getting ring objects
       _ringObjectsSuscription = this->create_subscription<delta_interfaces::msg::RingObjects>(
         "ring_objects", 1000, std::bind(&MissionController::receiveRingObjectsUpdate, this, std::placeholders::_1));
       // getting cylinder objects
       _cylinderObjectsSuscription = this->create_subscription<delta_interfaces::msg::CylinderObjects>(
         "cylinder_objects", 1000, std::bind(&MissionController::receiveCylinderObjectsUpdate, this, std::placeholders::_1));
         
       // say things
       _sayTextPublisher = this->create_publisher<delta_interfaces::msg::SayText>("say_text", 10);
        
      // receiving job status
      _jobStatusSuscription = this->create_subscription<delta_interfaces::msg::JobStatus>(
        "job_status", 1, std::bind(&MissionController::receiveJobStatusUpdate, this, std::placeholders::_1));
        
      // sending greeter jobs
      _greeterJobPublisher = this->create_publisher<delta_interfaces::msg::GreeterJob>("greeter_job", 1);
      
      // sending parking jobs
      _parkingJobPublisher = this->create_publisher<delta_interfaces::msg::ParkingJob>("parking_job", 1);
      
      // sending explorer jobs
      _explorerJobPublisher = this->create_publisher<delta_interfaces::msg::ExplorerJob>("explorer_job", 1);
      
      // sending ml_identifier jobs
      _mlJobPublisher = this->create_publisher<delta_interfaces::msg::MonalisaJob>("monalisa_job", 1);
      
      _decisionTimer = this->create_wall_timer(500ms, std::bind(&MissionController::makeDecision, this));
      
      // start witch exploring so that the explorer node can undock the robot first
      switchStatus(continueExploring);
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
    mutable vector<string> _monaLisaStillToCheckIds;
    // ring objects
    rclcpp::Subscription<delta_interfaces::msg::RingObjects>::SharedPtr _ringObjectsSuscription;
    mutable map<string, bool> _knownRingObjectIds = {};
    // cylinder objects
    rclcpp::Subscription<delta_interfaces::msg::CylinderObjects>::SharedPtr _cylinderObjectsSuscription;
    mutable map<string, bool> _knownCylinderObjectIds = {};
    
    // saying things
    rclcpp::Publisher<delta_interfaces::msg::SayText>::SharedPtr _sayTextPublisher;
    mutable vector<string> _textsToSay;
    
    // sending jobs to servants stuff
    mutable bool _servantReceivedJob = false;
    mutable bool _servantHasFinishedJob = false;
    mutable string _sentJobId = ""; // the id of the job that was last sent
    mutable int _jobCounter = 0; // makes sure each job has an individual id
    
    // task 3
    mutable bool _ringColorKnown = false;
    mutable string _ringColor = "";
    mutable bool _cylinderApproached = false;
    mutable bool _qrCodeScanned = false;
    mutable map<string, double> _knownRingCoordinateX = {};
    mutable map<string, double> _knownRingCoordinateY = {};
    mutable int _answerCountGreen = 0;
    mutable int _answerCountRed = 0;
    mutable int _answerCountBlue = 0;
    mutable int _answerCountBlack = 0;
    mutable bool _missionComplete = false;
    
    // getting job status updates
    rclcpp::Subscription<delta_interfaces::msg::JobStatus>::SharedPtr _jobStatusSuscription;
    
    // explorer stuff
    rclcpp::Publisher<delta_interfaces::msg::ExplorerJob>::SharedPtr _explorerJobPublisher;
    
    // mona lisa check stuff
    mutable string _nextMonalisaToCheckId = "";
    rclcpp::Publisher<delta_interfaces::msg::MonalisaJob>::SharedPtr _mlJobPublisher;
    
    // greeting stuff
    mutable string _nextPersonToGreetId = "";
    rclcpp::Publisher<delta_interfaces::msg::GreeterJob>::SharedPtr _greeterJobPublisher;
    
    // parking stuff
    mutable double _nextParkPositionX = 0;
    mutable double _nextParkPositionY = 0;
    rclcpp::Publisher<delta_interfaces::msg::ParkingJob>::SharedPtr _parkingJobPublisher;


    void initNewJobId() {
        _sentJobId = "mission_control_job"+ to_string(_jobCounter);
        _jobCounter++;
        _servantReceivedJob = false;
        _servantHasFinishedJob = false;
    }
    
    
    void makeDecision() {
    
        // send text to speaker if there is something to say
        if (_textsToSay.size() > 0) {
            string text = _textsToSay.back(); //[lastIndex];
            _textsToSay.pop_back();  //.erase(lastIndex);
            sendSayTextMsg(text);
        }
        
        if (_missionComplete) {
            //_decisionTimer->cancel();
            return;
        }
        
        
        // the robot is ready to perform the next best action
        if (_myStatus == ready) {

            if (personToGreetAvailable() && !_ringColorKnown) {
                // need to greet people to know ring color
                switchStatus(greetingPerson);
                
            } else if (_ringColorKnown && !_cylinderApproached && _knownRingCoordinateY.count(_ringColor) != 0) {
                // need to park and aproach cylinder
                _nextParkPositionX = _knownRingCoordinateX[_ringColor];
                _nextParkPositionY = _knownRingCoordinateY[_ringColor];
                switchStatus(parking);
                
            } else if (_cylinderApproached && !_qrCodeScanned) {
                // need to scan qr code now
                switchStatus(scanningQr);
                
            } else if (_qrCodeScanned && knowMonaLisaToCheck()) {
                // check next mona lisa
                switchStatus(approachingMonalisa);
                
            }else {
                // i dont need to do anything else so i go exlore =)
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
                if (personToGreetAvailable() && !_ringColorKnown) {
                    // robot has to greet -> switch to stop exploration status
                    RCLCPP_INFO(this->get_logger(), "continueExploring : need to greet someone to find out ring color, stopping exploration");
                    switchStatus(stopExploring);
                
                } else if (_ringColorKnown && !_cylinderApproached && _knownRingCoordinateY.count(_ringColor) != 0) {
                    // robot has to park and approach cylinder -> switch to stop exploration status
                    RCLCPP_INFO(this->get_logger(), "continueExploring : need to park and approach cylinder, stopping exploration");
                    switchStatus(stopExploring);
                    
                } else if (_cylinderApproached && !_qrCodeScanned) {
                    // robot has to scan the qr code now -> switch to stop exploration status
                    RCLCPP_INFO(this->get_logger(), "continueExploring : need scan qr code now, stopping exploration");
                    switchStatus(stopExploring);
                    
                } else if (_qrCodeScanned && knowMonaLisaToCheck()) {
                    // robot has to check the next mona lisa -> switch to stop exploration status
                    RCLCPP_INFO(this->get_logger(), "continueExploring : need to check out next mona lisa now, stopping exploration");
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
                switchStatus(ready);
            }
        }
        
        
        // the robot is currently parking
        else if (_myStatus == parking) {
        
            // delta_parking has not received job yet -> sending job
            if (!_servantReceivedJob) {
                RCLCPP_INFO(this->get_logger(), "parking : sending job");
                sendParkingJob();
            }
            
            // delta_parking has already received job but not finished yet
            else if (!_servantHasFinishedJob) {
                // give delta_parking time to park
                RCLCPP_INFO(this->get_logger(), "parking : waiting for delta_parker to park");
                
            } else {                
                // parking is done. ready for new task
                RCLCPP_INFO(this->get_logger(), "parking : finished. getting ready for new task");
                _cylinderApproached = true;
                switchStatus(ready);
            }
        }
        
        else if (_myStatus == scanningQr) {
            // has not received job yet -> sending job
            if (!_servantReceivedJob) {
                RCLCPP_INFO(this->get_logger(), "scanningQr : sending job");
                sendMonalisaJob(true);
            }
            
            // has already received job but not finished yet
            else if (!_servantHasFinishedJob) {
                // wait till scan finished
                RCLCPP_INFO(this->get_logger(), "scanningQr : waiting for scan to finish");
                
            } else {
                // sacnning done. ready for new task
                RCLCPP_INFO(this->get_logger(), "scanningQr : finished. getting ready for new task");
                _qrCodeScanned = true;
                switchStatus(ready);
            }
        }
        
        else if (_myStatus == approachingMonalisa) {
            // has not received job yet -> sending job
            if (!_servantReceivedJob) {
                RCLCPP_INFO(this->get_logger(), "approachingMonalisa : sending job");
                sendApproachMonaLisaJob();
            }
            
            // has already received job but not finished yet
            else if (!_servantHasFinishedJob) {
                // wait till scan finished
                RCLCPP_INFO(this->get_logger(), "approachingMonalisa : waiting for finish");
                
            } else {
                // sacnning done. ready for new task
                RCLCPP_INFO(this->get_logger(), "approachingMonalisa : finished. getting ready for new task");
                switchStatus(checkMonalisa);
            }
        }
        
        else if (_myStatus == checkMonalisa) {
            // has not received job yet -> sending job
            if (!_servantReceivedJob) {
                RCLCPP_INFO(this->get_logger(), "checkMonalisa : sending job");
                sendMonalisaJob(false);
            }
            
            // has already received job but not finished yet
            else if (!_servantHasFinishedJob) {
                // wait till scan finished
                RCLCPP_INFO(this->get_logger(), "checkMonalisa : waiting for finish");
                
            } else {
                // sacnning done. ready for new task
                RCLCPP_INFO(this->get_logger(), "checkMonalisa : finished. getting ready for new task");
                switchStatus(ready);
            }
        }
    }
    
    void sayText(string textToSay) {
        _textsToSay.insert(_textsToSay.begin(), textToSay);
    }
    
    void sendSayTextMsg(string textToSay) {
        auto message = delta_interfaces::msg::SayText();
        message.text = textToSay;
        _sayTextPublisher->publish(message);
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
            sendGreeterJob()
            ;
        } else if (newStatus == parking) {
            initNewJobId();
            RCLCPP_INFO(this->get_logger(), "parking : sending job");
            sendParkingJob();
            
        } else if(newStatus == scanningQr) {
            initNewJobId();
            RCLCPP_INFO(this->get_logger(), "scanningQr : sending job");
            sendMonalisaJob(true);
            
        } else if (newStatus == approachingMonalisa) {
            initNewJobId();
            _nextMonalisaToCheckId = popMonaLisaToCheckAndReturnId();
            RCLCPP_INFO(this->get_logger(), "approachingMonalisa : sending job");
            sendApproachMonaLisaJob();
        
        } else if(newStatus == checkMonalisa) {
            initNewJobId();
            RCLCPP_INFO(this->get_logger(), "checkMonalisa : sending job");
            sendMonalisaJob(false);
        }
    }
    
    bool personToGreetAvailable() {
        return _peopleStillToGreetIds.size() > 0;
    }
    
    bool knowMonaLisaToCheck() {
        return _monaLisaStillToCheckIds.size() > 0;
    }
    
    string popPersonToGreetAndReturnId() {
        if (!personToGreetAvailable()) RCLCPP_INFO(this->get_logger(), "ERROR: there are no people left in _peopleStillToGreetIds");
        
        //int lastIndex = _peopleStillToGreetIds.size() - 1; // get and remove last element
        string personToGreetId = _peopleStillToGreetIds.back(); //[lastIndex];
        _peopleStillToGreetIds.pop_back();  //.erase(lastIndex);
        
        return personToGreetId;
    }
    
    string popMonaLisaToCheckAndReturnId() {
        if (!knowMonaLisaToCheck()) RCLCPP_INFO(this->get_logger(), "ERROR: _monaLisaStillToCheckIds is empty. can not get next mona lisa");
        
        string monaLisaToCheckId = _monaLisaStillToCheckIds.back(); //[lastIndex];
        _monaLisaStillToCheckIds.pop_back();  //.erase(lastIndex);
        
        return monaLisaToCheckId;
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
            message.person_id = _nextPersonToGreetId;
            message.talk_to_person = true;
            _greeterJobPublisher->publish(message);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "ERROR: person that has to be greeted is not in the list of all level object");
    }
    
    void sendApproachMonaLisaJob() {
        for (int i = 0; i < _levelObjects.numberOfObjects; i++) {
            if (_levelObjects.id[i] != _nextMonalisaToCheckId) continue;
            
            auto message = delta_interfaces::msg::GreeterJob();
            message.position_x = _levelObjects.x[i];
            message.position_y = _levelObjects.y[i];
            message.position_z = _levelObjects.z[i];
            message.rotation = _levelObjects.rot[i];
            message.job_id = _sentJobId;
            message.person_id = _nextMonalisaToCheckId;
            message.talk_to_person = false;
            _greeterJobPublisher->publish(message);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "ERROR: person that has to be greeted is not in the list of all level object");
    }
    
    void sendParkingJob() {
        auto message = delta_interfaces::msg::ParkingJob();
        message.position_x = _nextParkPositionX;
        message.position_y = _nextParkPositionY;
        message.job_id = _sentJobId;
        _parkingJobPublisher->publish(message);
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
    
    void sendMonalisaJob(bool scanQr) {
        auto message = delta_interfaces::msg::MonalisaJob();
        message.scan_qr = scanQr;
        message.job_id = _sentJobId;
        _mlJobPublisher->publish(message);
    }
    
    void receiveJobStatusUpdate(const delta_interfaces::msg::JobStatus & msg) const {
    
        // the job has arrived
        if (msg.job_id == _sentJobId) {
            _servantReceivedJob = true;
        }
        
        // the job has arrived and is eighter not processed anymore -> finished
        if (msg.job_id == _sentJobId && (!msg.acting) && !_servantHasFinishedJob) {
            
            if (_myStatus == greetingPerson) {
                if (msg.result_string1 == "green" || msg.result_string2 == "green") _answerCountGreen++;
                if (msg.result_string1 == "red" || msg.result_string2 == "red") _answerCountRed++;
                if (msg.result_string1 == "blue" || msg.result_string2 == "blue") _answerCountBlue++;
                if (msg.result_string1 == "black" || msg.result_string2 == "black") _answerCountBlack++;
                if (_answerCountGreen >= 2) {
                    _ringColorKnown = true;
                    _ringColor = "green";
                } else if (_answerCountRed >= 2) {
                    _ringColorKnown = true;
                    _ringColor = "red";
                } else if (_answerCountBlue >= 2) {
                    _ringColorKnown = true;
                    _ringColor = "blue";
                } else if (_answerCountBlack >= 2) {
                    _ringColorKnown = true;
                    _ringColor = "black";
                }
            }
            
            if (_myStatus == checkMonalisa) {
                if (msg.result_bool) {
                    RCLCPP_INFO(this->get_logger(), ">>> MISSION COMPLETE <<<");
                    _textsToSay.insert(_textsToSay.begin(), "It is HER. The ONE and ONLY mona lisa! I can finally steal her. Ha ha ha!");
                    _textsToSay.insert(_textsToSay.begin(), "This was such an exciting mission! I still remember when I met PERSON ONE. I said. hello PERSON ONE. And later during the parking I got so nervous. But everything worked out perfectly. So now it is time to say goodbye. Goodbye.");
                    _missionComplete = true;
                }
            }
            
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
              
              
              if (id.find("person") != std::string::npos) { // 'person' is part of the id
                  _peopleStillToGreetIds.push_back(id); // add it to list of people that the robot still needs to greet
              } else {
                  // it must be a mona lisa! wow!!!
                  _monaLisaStillToCheckIds.push_back(id);
                  
              }
              
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
    
    
    void receiveRingObjectsUpdate(const delta_interfaces::msg::RingObjects & msg) const {
      for (int i = 0; i < static_cast<int>(msg.id.size()); i++) {
          string id = msg.id[i];
          if (_knownRingObjectIds.count(id) == 0) { // check if _knownRingObjectIds is not containing msg.id[i] yet
              _knownRingObjectIds[id] = true; // add to map to keep track which object ids are known already
              
              RCLCPP_INFO(this->get_logger(), ">>> I heard a new ring id: '%s'", id.c_str());
              _textsToSay.insert(_textsToSay.begin(), "hello "+ msg.color[i] + " ring");
              
              
              _knownRingCoordinateX[msg.color[i]] = msg.position_x[i];
              _knownRingCoordinateY[msg.color[i]] = msg.position_y[i];
          }
          
      }
    }
    
    void receiveCylinderObjectsUpdate(const delta_interfaces::msg::CylinderObjects & msg) const {
      for (int i = 0; i < static_cast<int>(msg.id.size()); i++) {
          string id = msg.id[i];
          if (_knownCylinderObjectIds.count(id) == 0) { // check if _knownCylinderObjectIds is not containing msg.id[i] yet
              _knownCylinderObjectIds[id] = true; // add to map to keep track which object ids are known already
              
              RCLCPP_INFO(this->get_logger(), ">>> I heard a new cylinder id: '%s'", id.c_str());
              _textsToSay.insert(_textsToSay.begin(), "hello "+ msg.color[i] + " cylinder");
          }
          
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
