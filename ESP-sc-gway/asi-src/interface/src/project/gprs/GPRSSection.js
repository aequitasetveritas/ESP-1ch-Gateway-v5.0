import React, { Component } from 'react';
import { Redirect, Switch } from 'react-router-dom'

import { PROJECT_PATH } from '../../constants/Env';
import MenuAppBar from '../../components/MenuAppBar';
import AuthenticatedRoute from '../../authentication/AuthenticatedRoute';
import GPRSTabStatus from './GPRSTabStatus';
import GPRSTabConfig from './GPRSTabConfig';

import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';

class GPRSSection extends Component {

  handleTabChange = (event, path) => {
    this.props.history.push(path);
  };

  render() {
    return (
      <MenuAppBar sectionTitle="GPRS">
        <Tabs value={this.props.match.url} onChange={this.handleTabChange} indicatorColor="primary" textColor="primary" variant="fullWidth">
          <Tab value={`/${PROJECT_PATH}/gprs/status`} label="Estado" />
          <Tab value={`/${PROJECT_PATH}/gprs/config`} label="Configuración" />
        </Tabs>
        <Switch>
          <AuthenticatedRoute exact path={`/${PROJECT_PATH}/gprs/status`} component={GPRSTabStatus} />
          <AuthenticatedRoute exact path={`/${PROJECT_PATH}/gprs/config`} component={GPRSTabConfig} />
          <Redirect to={`/${PROJECT_PATH}/gprs/status`}  />
        </Switch>
      </MenuAppBar>
    )
  }        

}

export default GPRSSection;
