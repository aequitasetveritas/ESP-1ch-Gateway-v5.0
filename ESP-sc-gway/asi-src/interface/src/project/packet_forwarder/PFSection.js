import React, { Component } from 'react';
import { Redirect, Switch } from 'react-router-dom'

import { PROJECT_PATH } from '../../constants/Env';
import MenuAppBar from '../../components/MenuAppBar';
import AuthenticatedRoute from '../../authentication/AuthenticatedRoute';
import PFTabConfig from './PFTabConfig';

import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';

class PFSection extends Component {

  handleTabChange = (event, path) => {
    this.props.history.push(path);
  };

  render() {
    return (
      <MenuAppBar sectionTitle="Packet Forwarder">
        <Tabs value={this.props.match.url} onChange={this.handleTabChange} indicatorColor="primary" textColor="primary" variant="fullWidth">
          <Tab value={`/${PROJECT_PATH}/pf/config`} label="Configuración" />
        </Tabs>
        <Switch>
          <AuthenticatedRoute exact path={`/${PROJECT_PATH}/pf/config`} component={PFTabConfig} />
          <Redirect to={`/${PROJECT_PATH}/pf/config`}  />
        </Switch>
      </MenuAppBar>
    )
  }        

}


          // <Tab value={`/${PROJECT_PATH}/lora/status`} label="Estado" />
          //<AuthenticatedRoute exact path={`/${PROJECT_PATH}/lora/status`} component={LoraTabStatus} />
export default PFSection;
