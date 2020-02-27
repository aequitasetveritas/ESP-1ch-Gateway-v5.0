import React, { Component } from 'react';
import { Redirect, Switch } from 'react-router-dom'

import { PROJECT_PATH } from '../constants/Env';
import MenuAppBar from '../components/MenuAppBar';
import AuthenticatedRoute from '../authentication/AuthenticatedRoute';
import LoraTabStatus from './LoraTabStatus';
import LoraTabConfig from './LoraTabConfig';

import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';

class LoraSection extends Component {

  handleTabChange = (event, path) => {
    this.props.history.push(path);
  };

  render() {
    return (
      <MenuAppBar sectionTitle="Lora">
        <Tabs value={this.props.match.url} onChange={this.handleTabChange} indicatorColor="primary" textColor="primary" variant="fullWidth">
          <Tab value={`/${PROJECT_PATH}/lora/status`} label="Estado" />
          <Tab value={`/${PROJECT_PATH}/lora/config`} label="Configuración Radio" />
        </Tabs>
        <Switch>
          <AuthenticatedRoute exact path={`/${PROJECT_PATH}/lora/status`} component={LoraTabStatus} />
          <AuthenticatedRoute exact path={`/${PROJECT_PATH}/lora/config`} component={LoraTabConfig} />
          <Redirect to={`/${PROJECT_PATH}/lora/status`}  />
        </Switch>
      </MenuAppBar>
    )
  }        

}

export default LoraSection;
