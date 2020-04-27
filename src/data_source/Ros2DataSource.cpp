
/*
 * Ros2DataSource.cpp
 * 
 * @author Lin Azan (lin@cogniteam.com)
 * @date 2020-03-15
 * @copyright Cogniteam (c) 2020
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2010-2020 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */

#include <cognitao_ros2/data_source/Ros2DataSource.h>


using std::placeholders::_1;

using namespace std;


Ros2DataSource::Ros2DataSource(){


    if (!rclcpp::ok()){
        rclcpp::init(0, nullptr);
    }
   
    g_node_ = rclcpp::Node::make_shared("cognitao_ros2");

    event_pub_ =
        g_node_->create_publisher<cognitao_ros2::msg::Event>("/wme/in", 1000);    

    event_sub_ = g_node_->create_subscription<cognitao_ros2::msg::Event>(
      "/wme/in", 1000, std::bind(&Ros2DataSource::onDataSourceEvent, this, _1));
 
    
    spinThread_ = std::thread(&Ros2DataSource::doSpin, this);
    spinThread_.detach();
}

bool Ros2DataSource::publishUpdateEvent(const string &name, const string &value){
    
    cognitao_ros2::msg::Event eventMsg;

    eventMsg.key = name;

    eventMsg.value = value;

    event_pub_->publish(eventMsg);

    return true;
}
void Ros2DataSource::onDataSourceEvent(const cognitao_ros2::msg::Event::SharedPtr msg){

    DataSource::variableUpdated(msg->key, msg->value);
}


void Ros2DataSource::doSpin(){
	 
	rclcpp::spin(g_node_);
	
}
