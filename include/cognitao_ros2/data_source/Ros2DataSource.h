/*
 * RosDataSource.h
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


#ifndef ROS2_DATA_SOURCE_H_
#define ROS2_DATA_SOURCE_H_


#include <cognitao/CogniTao.h>
#include <rclcpp/rclcpp.hpp>
#include <cognitao_ros2/msg/event.hpp>
#include <cognitao/data_sources/DataSource.h>

#include "std_msgs/msg/string.hpp"

// #include <cognitao/CogniTao.h>

using namespace std;


/**
 * manage the WM
 */
class Ros2DataSource :public DataSource{

public:

    Ros2DataSource();

    virtual ~Ros2DataSource(){};

public:

    static DataSource::Ptr create(){
        return DataSource::Ptr(new Ros2DataSource());
    }

protected:

    /**
     * @brief publish events
     * @return bool
     */
    virtual bool publishUpdateEvent(const string &name,
                                    const string &value);

    /**
     * @brief updates the world model
     * @return bool
     */
    void  onDataSourceEvent(const cognitao_ros2::msg::Event::SharedPtr msg);

    void doSpin();


private:

    rclcpp::Subscription<cognitao_ros2::msg::Event>::SharedPtr event_sub_;

	rclcpp::Publisher<cognitao_ros2::msg::Event>::SharedPtr event_pub_;

	std::thread spinThread_;

	rclcpp::Node::SharedPtr g_node_ = nullptr;       

};



#endif /* ROS2_DATA_SOURCE_H_ */

