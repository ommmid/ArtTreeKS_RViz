#ifndef PLUCKERCOORDINATE_H
#define PLUCKERCOORDINATE_H

#include <iostream>
#include <vector>

template<typename vectorType>
struct PluckerCoordinate
{
public:
  PluckerCoordinate();
  PluckerCoordinate(const std::vector<vectorType> &dir, const std::vector<vectorType> &mom);
  std::vector<vectorType> getDirection() const;
  std::vector<vectorType> getMoment() const;
  std::vector<vectorType> getPerpPoint();

private:
  std::vector<vectorType> direction;
  std::vector<vectorType> moment;
  std::vector<vectorType> perpPoint;

  std::vector<vectorType> calculatePerpPoint(); // get perpendicular point on the line

};

template<typename vectorType>
PluckerCoordinate<vectorType>::PluckerCoordinate(){

}

template<typename vectorType>
PluckerCoordinate<vectorType>::PluckerCoordinate(const std::vector<vectorType> &dir, const std::vector<vectorType> &mom){
    direction = dir;
    moment = mom;
    perpPoint = calculatePerpPoint();
}

template<typename vectorType>
std::vector<vectorType> PluckerCoordinate<vectorType>::getDirection() const{
      return direction;
}

template<typename vectorType>
std::vector<vectorType> PluckerCoordinate<vectorType>::getMoment() const{
      return moment;
}

template<typename vectorType>
std::vector<vectorType> PluckerCoordinate<vectorType>::getPerpPoint(){
    return perpPoint;
}

template<typename vectorType>
std::vector<vectorType> PluckerCoordinate<vectorType>::calculatePerpPoint(){ // get perpendicular point on the line
  // perp = direction x moment
   std::vector<vectorType> perp;
   perp.push_back(direction[1]*moment[2] - direction[2]*moment[1]);
   perp.push_back(direction[2]*moment[0] - direction[0]*moment[2]);
   perp.push_back(direction[0]*moment[1] - direction[1]*moment[0]);

return perp;
}


#endif // PLUCKERCOORDINATE_H
